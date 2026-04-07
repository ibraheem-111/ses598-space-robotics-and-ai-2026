#!/usr/bin/env python3

import math
import time
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from px4_msgs.msg import (
    VehicleOdometry,
    OffboardControlMode,
    VehicleCommand,
    TrajectorySetpoint,
    BatteryStatus,
)
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import numpy as np

from message_filters import ApproximateTimeSynchronizer, Subscriber

from logging import Formatter


class CylinderMission(Node):
    def __init__(self):
        super().__init__('cylinder_mission_node')

        # ---------------------------------------------
        # Parameters
        # ---------------------------------------------
        self.declare_parameter('orbit_center_x', 0.0)          # odom frame
        self.declare_parameter('orbit_center_y', 0.0)          # odom frame
        self.declare_parameter('radius', 5.0)                  # meters
        self.declare_parameter('takeoff_altitude_ned', -5.0)   # PX4-style NED z
        self.declare_parameter('orbit_min_z_ned', -20.0)       # more negative = higher in NED
        self.declare_parameter('tangential_speed', 0.18)       # m/s around cylinder
        self.declare_parameter('vertical_speed_ned', float('nan'))
        self.declare_parameter('orbit_climb_step_ned', float('nan'))  # legacy alias, now interpreted as m/s
        self.declare_parameter('tangential_accel', 0.06)       # m/s^2 ramp for smooth start
        self.declare_parameter('vertical_accel', 0.02)         # m/s^2 ramp for smooth climb
        self.declare_parameter('control_period', 0.05)         # 20 Hz setpoint stream
        self.declare_parameter('circle_entry_speed', 0.30)     # m/s while joining circle
        self.declare_parameter('circle_entry_tolerance', 0.25) # meters
        self.declare_parameter('yaw_inward', True)

        # ---------------------------------------------
        # PX4 / Offboard QoS
        # ---------------------------------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------------------------------------
        # Publishers
        # ---------------------------------------------
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )

        # ---------------------------------------------
        # Subscribers
        # ---------------------------------------------
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile
        )

        self.caminfo_sub = self.create_subscription(
            CameraInfo, '/drone/camera_info', self.caminfo_callback, 10
        )

        self.rgb_sub = Subscriber(self, Image, '/drone/front_rgb')
        self.depth_sub = Subscriber(self, Image, '/drone/front_depth')
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        self.marker_pose_sub = self.create_subscription(
            String, '/aruco/marker_pose', self.aruco_cb, 10
        )

        self.battery_sub = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status_v1', self.battery_cb, qos_profile
        )

        # ---------------------------------------------
        # Timing / state
        # ---------------------------------------------
        self.control_period = float(self.get_parameter('control_period').value)
        self.timer = self.create_timer(self.control_period, self.timer_callback)
        self.offboard_setpoint_counter = 0
        self.offboard_warmup_cycles = max(10, math.ceil(1.2 / self.control_period))
        self.state = 'WAIT_INTRINSICS'
        self.armed = False
        self.last_offboard_reassert_time = 0.0
        self.offboard_reassert_period_sec = 0.5

        # ---------------------------------------------
        # Vehicle state
        # ---------------------------------------------
        self.position = [0.0, 0.0, 0.0]  # vehicle_odometry frame (Gazebo-synced frame)
        self.odom_pose_frame = None
        self.bridge = CvBridge()

        # Camera intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ---------------------------------------------
        # Helix / spiral parameters (defined in odom frame)
        # ---------------------------------------------
        self.orbit_center_x = float(self.get_parameter('orbit_center_x').value)
        self.orbit_center_y = float(self.get_parameter('orbit_center_y').value)
        self.circle_radius = float(self.get_parameter('radius').value)
        self.takeoff_altitude_ned = float(self.get_parameter('takeoff_altitude_ned').value)
        self.orbit_min_z_ned = float(self.get_parameter('orbit_min_z_ned').value)
        self.tangential_speed_target = float(self.get_parameter('tangential_speed').value)
        self.tangential_accel = float(self.get_parameter('tangential_accel').value)
        self.vertical_accel = float(self.get_parameter('vertical_accel').value)
        self.circle_entry_speed = float(self.get_parameter('circle_entry_speed').value)
        self.circle_entry_tolerance = float(self.get_parameter('circle_entry_tolerance').value)
        self.yaw_inward = bool(self.get_parameter('yaw_inward').value)

        vertical_speed_param = float(self.get_parameter('vertical_speed_ned').value)
        legacy_vertical_speed_param = float(self.get_parameter('orbit_climb_step_ned').value)
        if math.isfinite(vertical_speed_param):
            self.vertical_speed_target_ned = vertical_speed_param
        elif math.isfinite(legacy_vertical_speed_param):
            self.vertical_speed_target_ned = legacy_vertical_speed_param
        else:
            self.vertical_speed_target_ned = -0.02

        self.altitude = self.takeoff_altitude_ned
        self.helix_theta = 0.0
        self.helix_z = self.takeoff_altitude_ned
        self.helix_initialized = False
        self.helix_entry_logged = False
        self.current_tangential_speed = 0.0
        self.current_vertical_speed_ned = 0.0
        self.aruco_hover_target_xy = None

        # ---------------------------------------------
        # Cylinder detection and measurement
        # ---------------------------------------------
        self.single_cylinder_mode = True
        self.measured_cylinders = []
        self.points_buffer = []
        self.sample_threshold = 10
        self.desired_distance = 15.0
        self.distance_tolerance = 0.3
        self.hover_start_time = None
        self.servo_start_time = None
        self.min_pixel_area = 5000
        self.detection_cooldown_until = 0.0

        # ---------------------------------------------
        # ArUco / mission transitions
        # ---------------------------------------------
        self.aruco_stable_required_sec = 0.2
        self.aruco_detection_loss_timeout_sec = 1.2
        self.aruco_first_seen_time = None
        self.aruco_last_seen_time = None
        self.aruco_detection_count = 0
        self.aruco_detection_count_required = 3
        self.markers = {}
        self.land_target = None
        self.aruco_hover_start_time = None

        # ---------------------------------------------
        # Logging / battery
        # ---------------------------------------------
        self.start_time = None
        self.battery_percent = None
        self.battery_at_mission_start = None
        self.battery_at_mission_end = None

    # ---------------------------------------------
    # Utility helpers
    # ---------------------------------------------
    @staticmethod
    def wrap_pi(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def move_scalar_toward(current, target, max_step):
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + math.copysign(max_step, delta)

    @staticmethod
    def move_xy_toward(current_x, current_y, target_x, target_y, max_step):
        dx = target_x - current_x
        dy = target_y - current_y
        dist = math.hypot(dx, dy)
        if dist <= max_step or dist < 1e-9:
            return target_x, target_y
        scale = max_step / dist
        return current_x + dx * scale, current_y + dy * scale

    @staticmethod
    def ramp_toward(current, target, max_delta):
        delta = target - current
        if abs(delta) <= max_delta:
            return target
        return current + math.copysign(max_delta, delta)

    # ---------------------------------------------
    # PX4 callbacks
    # ---------------------------------------------
    def battery_cb(self, msg):
        if not math.isnan(msg.volt_based_soc_estimate):
            self.battery_percent = msg.volt_based_soc_estimate

    def odom_cb(self, msg):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]
        self.odom_pose_frame = msg.pose_frame

    def caminfo_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.get_logger().info('Camera intrinsics received.')
        if self.caminfo_sub is not None:
            self.destroy_subscription(self.caminfo_sub)
            self.caminfo_sub = None

    # ---------------------------------------------
    # Frame conversion: odom frame -> PX4 local NED
    # ---------------------------------------------
    def odom_to_px4_ned(self, x_odom, y_odom, z_odom):
        # Per your measured relationship, odom x/y are swapped relative to PX4 local frame.
        # Signs intentionally unchanged.
        return float(y_odom), float(x_odom), float(z_odom)

    def yaw_to_center_from_odom(self, x_odom, y_odom):
        cx_ned, cy_ned, _ = self.odom_to_px4_ned(self.orbit_center_x, self.orbit_center_y, 0.0)
        px_ned, py_ned, _ = self.odom_to_px4_ned(x_odom, y_odom, 0.0)
        return math.atan2(cy_ned - py_ned, cx_ned - px_ned)

    # ---------------------------------------------
    # PX4 publishing
    # ---------------------------------------------
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(msg)

    def publish_odom_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=None):
        x_ned, y_ned, z_ned = self.odom_to_px4_ned(x, y, z)

        msg = TrajectorySetpoint()
        msg.position = [x_ned, y_ned, z_ned]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan') if yaw is None else float(yaw)
        msg.yawspeed = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_cmd_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
        self.armed = True

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info('Offboard mode command sent')

    # ---------------------------------------------
    # Helix state management
    # ---------------------------------------------
    def reset_helix(self):
        self.helix_initialized = False
        self.helix_entry_logged = False
        self.current_tangential_speed = 0.0
        self.current_vertical_speed_ned = 0.0
        self.helix_z = self.altitude

    def publish_smooth_helix_setpoint(self):
        cx = self.orbit_center_x
        cy = self.orbit_center_y
        px = float(self.position[0])
        py = float(self.position[1])
        pz = float(self.position[2])
        dt = self.control_period

        dx = px - cx
        dy = py - cy
        r_now = math.hypot(dx, dy)

        if r_now < 1e-6:
            entry_theta = 0.0
            ux = 1.0
            uy = 0.0
        else:
            entry_theta = math.atan2(dy, dx)
            ux = dx / r_now
            uy = dy / r_now

        entry_x = cx + self.circle_radius * ux
        entry_y = cy + self.circle_radius * uy
        entry_z = self.altitude

        if not self.helix_initialized:
            xy_step = self.circle_entry_speed * dt
            z_step = max(abs(self.vertical_speed_target_ned), 0.50) * dt

            x_cmd, y_cmd = self.move_xy_toward(px, py, entry_x, entry_y, xy_step)
            z_cmd = self.move_scalar_toward(pz, entry_z, z_step)
            yaw_cmd = self.yaw_to_center_from_odom(x_cmd, y_cmd) if self.yaw_inward else None
            self.publish_odom_trajectory_setpoint(x_cmd, y_cmd, entry_z, yaw=yaw_cmd)

            entry_error_xy = math.hypot(px - entry_x, py - entry_y)
            entry_error_z = abs(pz - entry_z)

            if not self.helix_entry_logged:
                self.get_logger().info(
                    f'Joining helix smoothly: current_radius={r_now:.2f} m, '
                    f'target_radius={self.circle_radius:.2f} m'
                )
                self.helix_entry_logged = True

            

            if entry_error_xy <= self.circle_entry_tolerance and entry_error_z <= 0.25:
                self.helix_initialized = True
                self.helix_theta = entry_theta
                self.helix_z = entry_z
                self.current_tangential_speed = 0.0
                self.current_vertical_speed_ned = 0.0
                self.get_logger().info('On entry circle. Beginning smooth helical scan.')
            else:
                self.get_logger().info(
                    f'Approaching entry circle: error_xy={entry_error_xy:.2f} m, '
                    f'error_z={entry_error_z:.2f} m'
                )
                self.get_logger().info(
                    f'command x = {x_cmd:.2f}, y={y_cmd:.2f}, z={z_cmd:.2f}, yaw={yaw_cmd:.2f}'
                )
                self.get_logger().info(
                    f'current radius={r_now:.2f} m, entry radius={math.hypot(entry_x - cx, entry_y - cy):.2f} m'
                    f'current x = {px:.2f}, y={py:.2f}, z={pz:.2f}, entry x={entry_x:.2f}, entry y={entry_y:.2f}, entry z={entry_z:.2f}'
                )
            return

        if pz<-12.0:
            self.circle_radius = 3.0
            self.vertical_speed_target_ned = -1.0
            


        self.current_tangential_speed = self.ramp_toward(
            self.current_tangential_speed,
            self.tangential_speed_target,
            self.tangential_accel * dt,
        )
        self.current_vertical_speed_ned = self.ramp_toward(
            self.current_vertical_speed_ned,
            self.vertical_speed_target_ned,
            self.vertical_accel * dt,
        )

        omega = self.current_tangential_speed / max(self.circle_radius, 0.1)
        self.helix_theta = self.wrap_pi(self.helix_theta + omega * dt)

        next_z = self.helix_z + self.current_vertical_speed_ned * dt
        if self.current_vertical_speed_ned < 0.0:
            self.helix_z = max(self.orbit_min_z_ned, next_z)
        else:
            self.helix_z = min(self.orbit_min_z_ned, next_z)

        x_cmd = cx + self.circle_radius * math.cos(self.helix_theta)
        y_cmd = cy + self.circle_radius * math.sin(self.helix_theta)
        z_cmd = self.helix_z
        yaw_cmd = self.yaw_to_center_from_odom(x_cmd, y_cmd) if self.yaw_inward else None

        self.get_logger().info(
            f"x={x_cmd:.2f}, y={y_cmd:.2f}, z={z_cmd:.2f}, yaw={yaw_cmd:.2f}"

        )

        self.publish_odom_trajectory_setpoint(x_cmd, y_cmd, z_cmd, yaw=yaw_cmd)

    # ---------------------------------------------
    # Callback: Synchronized Image + Depth
    # ---------------------------------------------
    def image_callback(self, rgb_msg, depth_msg):
        if time.time() < self.detection_cooldown_until:
            return

        if self.fx is None or self.fy is None:
            return

        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').astype(np.float32)
        depth[depth == 0] = np.nan

        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([0, 0, 110])
        upper_hsv = np.array([180, 40, 180])
        color_mask = cv2.inRange(hsv, lower_hsv, upper_hsv).astype(bool)

        depth_mask = np.logical_and(depth > 1.0, depth < 30.0)
        object_mask = np.logical_and(depth_mask, color_mask)

        object_mask = cv2.morphologyEx(
            object_mask.astype(np.uint8),
            cv2.MORPH_CLOSE,
            np.ones((5, 5), np.uint8),
        )

        contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = [c for c in contours if cv2.contourArea(c) > self.min_pixel_area]

        overlay = rgb.copy()

        if len(filtered) > 0:
            filtered.sort(key=cv2.contourArea, reverse=True)
            contour = filtered[0]
            x, y, w, h = cv2.boundingRect(contour)
            roi = depth[y:y + h, x:x + w]
            roi = roi[np.isfinite(roi)]

            if roi.size > 0:
                Z = float(np.median(roi))
                width_m = (w * Z) / self.fx
                height_m = (h * Z) / self.fy

                self.points_buffer.append((width_m, height_m, Z))

                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    overlay,
                    f'{width_m:.2f}m x {height_m:.2f}m',
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1,
                )

                if self.state == 'CIRCLE' and not self.single_cylinder_mode:
                    self.get_logger().info('Detected potential cylinder. Switching to SERVO state.')
                    self.state = 'SERVO'

        cv2.imshow('RGB Detection', overlay)
        cv2.imshow('Mask', object_mask.astype(np.uint8) * 255)
        cv2.waitKey(1)

    # ---------------------------------------------
    # ArUco detection and transformation to drone coordinates
    # ---------------------------------------------
    def aruco_cb(self, msg):
        import re

        match = re.match(r'Marker (\d+) detected at x:([-\d.]+)m, y:([-\d.]+)m, z:([-\d.]+)m', msg.data)
        if match:
            now = time.time()
            marker_id = int(match.group(1))
            x = float(match.group(2))
            y = float(match.group(3))
            z = float(match.group(4))

            drone_x = y
            drone_y = x
            drone_z = z
            self.markers[marker_id] = (drone_x, drone_y, drone_z)

            if self.aruco_last_seen_time is None or (now - self.aruco_last_seen_time) > self.aruco_detection_loss_timeout_sec:
                self.aruco_detection_count = 1
                self.aruco_first_seen_time = now
            else:
                self.aruco_detection_count += 1

            self.aruco_last_seen_time = now
            if self.aruco_first_seen_time is None:
                self.aruco_first_seen_time = now
            self.get_logger().info(f'Updated Marker {marker_id}: x={drone_x}, y={drone_y}, z={drone_z}')

        self.get_logger().info(f'ArUco detection count: {self.aruco_detection_count}')
        if self.aruco_detection_count >= 10 and self.position[2] < -12.5:
            self.get_logger().info('ArUco marker detected stably. Transitioning to ARUCO_HOVER.')
            self.state = 'ARUCO_HOVER'
            self.aruco_hover_start_time = time.time()


    def reassert_offboard_if_needed(self):
        now = time.time()
        if now - self.last_offboard_reassert_time >= self.offboard_reassert_period_sec:
            self.engage_offboard_mode()
            self.last_offboard_reassert_time = now

    def approach_entry_circle(self):
        # Find closest point on circle to current position
        dx = self.position[0] - self.orbit_center_x
        dy = self.position[1] - self.orbit_center_y
        angle_to_center = math.atan2(dy, dx)

        target_x = self.orbit_center_x + self.circle_radius * math.cos(angle_to_center)
        target_y = self.orbit_center_y + self.circle_radius * math.sin(angle_to_center)
        target_z = self.altitude

        target = [target_x, target_y, target_z]

        self.publish_odom_trajectory_setpoint(*target, yaw=None)

    # ---------------------------------------------
    # Main state machine
    # ---------------------------------------------
    def timer_callback(self):
        self.publish_offboard_control_mode()

        self.get_logger().info(f'State: {self.state}, Position: {self.position}, Battery: {self.battery_percent}')

        if self.state != 'WAIT_INTRINSICS':
            if not self.armed and self.offboard_setpoint_counter >= self.offboard_warmup_cycles:
                self.engage_offboard_mode()
                self.arm()
            self.offboard_setpoint_counter += 1

            # self.state="CIRCLE"

        if self.state == 'WAIT_INTRINSICS':
            if (self.fx is not None) and (self.fy is not None) and (self.battery_percent is not None):
                if self.battery_at_mission_start is None:
                    self.battery_at_mission_start = self.battery_percent
                    self.get_logger().info(
                        f'Locked battery_at_mission_start: {self.battery_at_mission_start:.4f}'
                    )

                self.get_logger().info('Intrinsics and battery OK. Moving to ARM_TAKEOFF.')
                self.state = 'CIRCLE'
                self.start_time = time.time()


        # elif self.state == "ARM_TAKEOFF":
        #     target = [0.0, 0.0, float(self.altitude)]
        #     self.publish_odom_trajectory_setpoint(*target, yaw=None)

        #     dx = self.position[0] - target[0]
        #     dy = self.position[1] - target[1]
        #     dz = self.position[2] - target[2]
        #     dist = math.sqrt(dx**2 + dy**2 + dz**2)

        #     if dist < 0.2:
        #         self.theta = math.atan2(
        #             self.position[1] - self.orbit_center_y,
        #             self.position[0] - self.orbit_center_x
        #         )

        #         dxc = self.position[0] - self.orbit_center_x
        #         dyc = self.position[1] - self.orbit_center_y
        #         current_radius = math.sqrt(dxc * dxc + dyc * dyc)
        #         radius_error = abs(current_radius - self.circle_radius)

        #         self.helix_joined = radius_error <= self.circle_entry_tolerance

        #         # self.get_logger().info(
        #         #     f"Vertical in-place takeoff complete. Switching to CIRCLE. "
        #         #     f"current_radius={current_radius:.2f}, "
        #         #     f"target_radius={self.circle_radius:.2f}, "
        #         #     f"radius_error={radius_error:.2f}, "
        #         #     f"joined={self.helix_joined}"
        #         # )

        #         self.state = "CIRCLE"

        if self.state == 'CIRCLE':
            print("In CIRCLE state")
            pass
            # self.publish_smooth_helix_setpoint()

        elif self.state == 'ARUCO_HOVER':
            self.reassert_offboard_if_needed()            
            target_x, target_y = self.markers[0][0], self.markers[0][1]
            target_z = self.orbit_min_z_ned
            # yaw_cmd = self.yaw_to_center_from_odom(target_x, target_y) if self.yaw_inward else None
            yaw_cmd = None
            self.publish_odom_trajectory_setpoint(target_x, target_y, target_z, yaw=yaw_cmd)

            if self.aruco_hover_start_time is None:
                pos_err = math.sqrt(
                    (self.position[0] - target_x) ** 2 +
                    (self.position[1] - target_y) ** 2 +
                    (self.position[2] - target_z) ** 2
                )
                if pos_err < 0.4:
                    self.aruco_hover_start_time = time.time()
                    self.get_logger().info('Reached hover point. Holding for 5 seconds...')

            elif time.time() - self.aruco_hover_start_time >= 5.0:
                self.get_logger().info('5s ArUco hover complete. Selecting marker...')
                self.state = 'ARUCO_SELECT'

        elif self.state == 'ARUCO_SELECT':
            if not self.markers:
                self.get_logger().warn('No ArUco markers available yet.')
                return

            dx, dy, dz = self.markers[0]
            self.land_target = [dx, dy, -abs(abs(self.orbit_min_z_ned) - dz)]
            self.get_logger().info(
                'Landing target selected based on ArUco marker: '
                f'x={dx:.2f}, y={dy:.2f}, z={self.land_target[2]:.2f}'
            )
            self.state = 'ARUCO_MOVE'

        elif self.state == 'ARUCO_MOVE':
            self.reassert_offboard_if_needed()
            if self.land_target is None:
                self.get_logger().warn('No landing target yet. Returning to ARUCO_SELECT.')
                self.state = 'ARUCO_SELECT'
                return

            x, y, z = self.land_target
            self.publish_odom_trajectory_setpoint(x=x, y=y, z=z, yaw=None)
            dist = math.sqrt(
                (self.position[0] - x) ** 2 +
                (self.position[1] - y) ** 2 +
                (self.position[2] - z) ** 2
            )
            if dist < 0.5:
                self.get_logger().info('Reached marker position. Initiating LAND.')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = 'ARUCO_LAND'

        elif self.state == 'ARUCO_LAND':
            self.get_logger().info('Landed successfully. Disarming...')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=0.0,
            )
            self.state = 'COMPLETE'

        elif self.state == 'COMPLETE':
            self.get_logger().info('Mission complete.')

            if self.battery_at_mission_end is None and self.battery_percent is not None:
                self.battery_at_mission_end = self.battery_percent
                self.get_logger().info(f'Captured battery_at_mission_end: {self.battery_at_mission_end:.4f}')

            if self.start_time is not None:
                mission_duration = time.time() - self.start_time
                self.get_logger().info(f'Mission Duration: {mission_duration:.2f} seconds')

                if self.battery_at_mission_start is not None and self.battery_at_mission_end is not None:
                    used = (self.battery_at_mission_start - self.battery_at_mission_end) * 100.0
                    self.get_logger().info(f'Battery Used: {used:.3f}%')
                else:
                    self.get_logger().warn('Missing start/end battery data!')

            self.state = 'DONE'

        elif self.state == 'DONE':
            rclpy.shutdown()

    

def main(args=None):
    rclpy.init(args=args)
    node = CylinderMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Interrupted, shutting down.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()