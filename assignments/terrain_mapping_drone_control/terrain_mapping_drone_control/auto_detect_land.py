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


class CylinderMission(Node):
    def __init__(self):
        super().__init__('cylinder_mission_node')

        self.declare_parameter('orbit_center_x', 0.0)
        self.declare_parameter('orbit_center_y', 5.0)
        self.declare_parameter('orbit_min_z_ned', -20.0)
        self.declare_parameter('radius', 5.0)
        self.declare_parameter('orbit_climb_step_ned', -0.005)
        self.declare_parameter('tangential_speed', 0.6)
        self.declare_parameter('takeoff_altitude_ned', -5.0)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )

        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile
        )

        self.caminfo_sub = self.create_subscription(
            CameraInfo, '/drone/camera_info', self.caminfo_callback, 10
        )

        self.rgb_sub = Subscriber(self, Image, '/drone/front_rgb')
        self.depth_sub = Subscriber(self, Image, '/drone/front_depth')
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.image_callback)

        self.state = 'WAIT_INTRINSICS'
        self.control_period = 0.01
        self.timer = self.create_timer(self.control_period, self.timer_callback)

        self.position = [0.0, 0.0, 0.0]
        self.odom_pose_frame = None
        self.bridge = CvBridge()
        self.armed = False
        self.offboard_warmup_counter = 0

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.circle_radius = float(self.get_parameter('radius').value)
        self.tangential_speed = float(self.get_parameter('tangential_speed').value)
        self.orbit_center_x = float(self.get_parameter('orbit_center_x').value)
        self.orbit_center_y = float(self.get_parameter('orbit_center_y').value)
        self.orbit_climb_step_ned = float(self.get_parameter('orbit_climb_step_ned').value)
        self.orbit_min_z_ned = float(self.get_parameter('orbit_min_z_ned').value)
        self.takeoff_altitude = float(self.get_parameter('takeoff_altitude_ned').value)
        self.altitude = self.takeoff_altitude
        self.theta = 0.0
        self.circle_started = False
        self.circle_entry_tolerance = 0.35

        self.home_x = 0.0
        self.home_y = 0.0

        self.single_cylinder_mode = True
        self.aruco_stable_required_sec = 2.0
        self.aruco_detection_loss_timeout_sec = 1.2
        self.aruco_first_seen_time = None
        self.aruco_last_seen_time = None
        self.aruco_detection_count = 0
        self.aruco_detection_count_required = 3
        self.last_offboard_reassert_time = 0.0
        self.offboard_reassert_period_sec = 0.5

        self.measured_cylinders = []
        self.points_buffer = []
        self.sample_threshold = 10
        self.desired_distance = 15.0
        self.distance_tolerance = 0.3
        self.hover_start_time = None
        self.servo_start_time = None
        self.min_pixel_area = 5000
        self.detection_cooldown_until = 0.0

        self.markers = {}
        self.land_target = None
        self.marker_pose_sub = self.create_subscription(
            String, '/aruco/marker_pose', self.aruco_cb, 10
        )
        self.aruco_hover_start_time = None

        self.start_time = None
        self.battery_percent = None
        self.battery_at_mission_start = None
        self.battery_at_mission_end = None
        self.battery_sub = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status_v1', self.battery_cb, qos_profile
        )

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

    def odom_to_px4_ned_position(self, x_odom, y_odom, z_odom):
        # Measured transform:
        # odom (x=5, y=0, z=0) -> px4 local NED (x=0, y=5, z=0)
        return -float(y_odom), float(x_odom), float(z_odom)

    def odom_vector_to_px4_ned(self, dx_odom, dy_odom, dz_odom=0.0):
        return -float(dy_odom), float(dx_odom), float(dz_odom)

    def yaw_toward_point_odom(self, from_x, from_y, to_x, to_y):
        dx_odom = float(to_x) - float(from_x)
        dy_odom = float(to_y) - float(from_y)
        dx_ned, dy_ned, _ = self.odom_vector_to_px4_ned(dx_odom, dy_odom, 0.0)
        return math.atan2(dy_ned, dx_ned)

    def publish_trajectory_setpoint_odom(self, x, y, z, yaw=None):
        x_ned, y_ned, z_ned = self.odom_to_px4_ned_position(x, y, z)

        msg = TrajectorySetpoint()
        msg.position = [float(x_ned), float(y_ned), float(z_ned)]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.jerk = [float('nan'), float('nan'), float('nan')]
        msg.yaw = float('nan') if yaw is None else float(yaw)
        msg.yawspeed = float('nan')
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(msg)

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

        if filtered:
            filtered.sort(key=cv2.contourArea, reverse=True)
            contour = filtered[0]
            x, y, w, h = cv2.boundingRect(contour)
            roi = depth[y:y + h, x:x + w]
            roi = roi[np.isfinite(roi)]

            if roi.size > 0:
                z_depth = float(np.median(roi))
                width_m = (w * z_depth) / self.fx
                height_m = (h * z_depth) / self.fy

                self.points_buffer.append((width_m, height_m, z_depth))

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
            self.get_logger().info(
                f'Updated Marker {marker_id}: x={drone_x:.2f}, y={drone_y:.2f}, z={drone_z:.2f}'
            )

    def aruco_is_stably_detected(self):
        if self.aruco_last_seen_time is None:
            return False

        now = time.time()
        if now - self.aruco_last_seen_time > self.aruco_detection_loss_timeout_sec:
            self.aruco_first_seen_time = None
            self.aruco_detection_count = 0
            return False

        if self.aruco_first_seen_time is None:
            self.aruco_first_seen_time = self.aruco_last_seen_time
            return False

        enough_time = (now - self.aruco_first_seen_time) >= self.aruco_stable_required_sec
        enough_hits = self.aruco_detection_count >= self.aruco_detection_count_required
        return enough_time and enough_hits

    def reassert_offboard_if_needed(self):
        now = time.time()
        if now - self.last_offboard_reassert_time >= self.offboard_reassert_period_sec:
            self.engage_offboard_mode()
            self.last_offboard_reassert_time = now

    def maybe_enter_offboard_and_arm(self):
        if self.state == 'WAIT_INTRINSICS' or self.armed:
            return

        if self.offboard_warmup_counter == 10:
            self.engage_offboard_mode()
            self.arm()

    def enter_circle_state(self):
        dx = self.position[0] - self.orbit_center_x
        dy = self.position[1] - self.orbit_center_y

        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            self.theta = 0.0
        else:
            self.theta = math.atan2(dy, dx)

        self.circle_started = False
        self.state = 'CIRCLE'

    def publish_spiral_setpoint(self):
        speed = abs(self.tangential_speed)
        if self.circle_radius <= 1e-6:
            omega = 0.0
        else:
            omega = speed / self.circle_radius
        if self.tangential_speed < 0.0:
            omega = -omega

        if not self.circle_started:
            entry_x = self.orbit_center_x + self.circle_radius * math.cos(self.theta)
            entry_y = self.orbit_center_y + self.circle_radius * math.sin(self.theta)
            entry_z = self.altitude

            yaw = self.yaw_toward_point_odom(
                self.position[0], self.position[1], self.orbit_center_x, self.orbit_center_y
            )
            self.publish_trajectory_setpoint_odom(entry_x, entry_y, entry_z, yaw=yaw)

            dist = math.sqrt(
                (self.position[0] - entry_x) ** 2 +
                (self.position[1] - entry_y) ** 2 +
                (self.position[2] - entry_z) ** 2
            )

            if dist < self.circle_entry_tolerance:
                self.circle_started = True
                self.get_logger().info('Reached spiral entry point. Starting spiral tracking.')
            return

        x_sp = self.orbit_center_x + self.circle_radius * math.cos(self.theta)
        y_sp = self.orbit_center_y + self.circle_radius * math.sin(self.theta)
        self.altitude = max(self.orbit_min_z_ned, self.altitude + self.orbit_climb_step_ned)
        z_sp = self.altitude

        yaw = self.yaw_toward_point_odom(
            self.position[0], self.position[1], self.orbit_center_x, self.orbit_center_y
        )
        self.publish_trajectory_setpoint_odom(x_sp, y_sp, z_sp, yaw=yaw)

        self.theta += omega * self.control_period
        if self.theta > math.pi:
            self.theta -= 2.0 * math.pi
        elif self.theta < -math.pi:
            self.theta += 2.0 * math.pi

    def timer_callback(self):
        self.publish_offboard_control_mode()

        if self.state != 'WAIT_INTRINSICS':
            self.offboard_warmup_counter += 1
        self.maybe_enter_offboard_and_arm()

        if self.state == 'WAIT_INTRINSICS':
            if self.fx is not None and self.fy is not None and self.battery_percent is not None:
                if self.battery_at_mission_start is None:
                    self.battery_at_mission_start = self.battery_percent
                    self.get_logger().info(
                        f'Locked battery_at_mission_start: {self.battery_at_mission_start:.4f}'
                    )

                self.home_x = self.position[0]
                self.home_y = self.position[1]
                self.altitude = self.takeoff_altitude
                self.start_time = time.time()
                self.offboard_warmup_counter = 0
                self.get_logger().info('Intrinsics and battery OK. Moving to ARM_TAKEOFF.')
                self.state = 'ARM_TAKEOFF'

        elif self.state == 'ARM_TAKEOFF':
            target_x = self.home_x
            target_y = self.home_y
            target_z = self.takeoff_altitude
            self.publish_trajectory_setpoint_odom(target_x, target_y, target_z, yaw=None)

            dist = math.sqrt(
                (self.position[0] - target_x) ** 2 +
                (self.position[1] - target_y) ** 2 +
                (self.position[2] - target_z) ** 2
            )

            if dist < 0.5:
                self.get_logger().info('Vertical in-place takeoff complete. Switching to CIRCLE.')
                self.enter_circle_state()

        elif self.state == 'CIRCLE':
            self.publish_spiral_setpoint()

            if self.aruco_is_stably_detected():
                self.get_logger().info(
                    f'ArUco stable for {self.aruco_stable_required_sec:.1f}s. '
                    f'Exiting spiral and moving to ARUCO_HOVER.'
                )
                self.aruco_hover_start_time = None
                self.reassert_offboard_if_needed()
                self.state = 'ARUCO_HOVER'

        elif self.state == 'SERVO':
            if self.servo_start_time is None:
                self.servo_start_time = time.time()

            current_distance = None
            if self.points_buffer:
                _, _, z_depth = self.points_buffer[-1]
                current_distance = z_depth

            if current_distance is None:
                if time.time() - self.servo_start_time > 5.0:
                    self.get_logger().warn('Object not found within timeout. Returning to CIRCLE.')
                    self.points_buffer.clear()
                    self.servo_start_time = None
                    self.enter_circle_state()
                else:
                    self.publish_trajectory_setpoint_odom(
                        self.position[0], self.position[1], self.altitude
                    )
            else:
                distance_error = self.desired_distance - current_distance
                drone_x, drone_y, _ = self.position
                gain = 0.5
                dx = distance_error * gain

                target_x = drone_x - dx
                target_y = drone_y
                target_z = self.altitude
                self.publish_trajectory_setpoint_odom(target_x, target_y, target_z)

                if abs(distance_error) < self.distance_tolerance:
                    self.get_logger().info('Reached ~15m from cylinder. Going to HOVER to measure.')
                    self.hover_start_time = time.time()
                    self.servo_start_time = None
                    self.state = 'HOVER'

        elif self.state == 'HOVER':
            self.publish_trajectory_setpoint_odom(
                self.position[0], self.position[1], self.altitude
            )

            if self.hover_start_time is None:
                self.hover_start_time = time.time()

            if time.time() - self.hover_start_time >= 7.0:
                self.get_logger().info('7s hover done. Checking measurement.')

                if self.single_cylinder_mode:
                    self.get_logger().info(
                        'Single-cylinder mode active. Skipping dimension matching and switching to ARUCO_HOVER.'
                    )
                    self.state = 'ARUCO_HOVER'
                    return

                if self.points_buffer:
                    widths, heights, depths = zip(*self.points_buffer)
                    median_w = statistics.median(widths)
                    median_h = statistics.median(heights)
                    self.get_logger().info(
                        f'[Cylinder Dimensions] Width={median_w:.2f} m, Height={median_h:.2f} m'
                    )

                    self.points_buffer.clear()

                    dimension_matched = False
                    tolerance = 0.3
                    for w_old, h_old in self.measured_cylinders:
                        if abs(w_old - median_w) < tolerance and abs(h_old - median_h) < tolerance:
                            dimension_matched = True
                            break

                    if dimension_matched:
                        self.get_logger().info(
                            'This cylinder matches a previously seen one. Mission done, landing.'
                        )
                        self.state = 'ARUCO_HOVER'
                    else:
                        self.get_logger().info('New cylinder dimension recorded. Resuming circle flight.')
                        self.measured_cylinders.append((median_w, median_h))
                        self.detection_cooldown_until = time.time() + 6.0
                        self.enter_circle_state()
                else:
                    self.get_logger().warn('No data in points_buffer. Resuming circle anyway.')
                    self.enter_circle_state()

        elif self.state == 'ARUCO_HOVER':
            self.reassert_offboard_if_needed()
            self.publish_trajectory_setpoint_odom(x=0.0, y=0.0, z=-20.0)

            if self.aruco_hover_start_time is None:
                if abs(self.position[2] + 20.0) < 0.3:
                    self.aruco_hover_start_time = time.time()
                    self.get_logger().info('Reached hover height. Holding for 5 seconds...')
            elif time.time() - self.aruco_hover_start_time >= 5.0:
                self.get_logger().info('5s ArUco hover complete. Selecting marker...')
                self.state = 'ARUCO_SELECT'

        elif self.state == 'ARUCO_SELECT':
            if len(self.markers) >= 1:
                best_marker_id = None
                min_z = float('inf')
                for marker_id, (mx, my, mz) in self.markers.items():
                    if mz < min_z:
                        min_z = mz
                        best_marker_id = marker_id
                if best_marker_id is not None:
                    dx, dy, dz = self.markers[best_marker_id]
                    self.land_target = [dx, dy, -abs(20.0 - dz)]
                    self.get_logger().info(
                        f'Selected Marker {best_marker_id} for landing at '
                        f'x={dx:.2f}, y={dy:.2f}, z={-abs(20.0 - dz):.2f}'
                    )
                    self.state = 'ARUCO_MOVE'

        elif self.state == 'ARUCO_MOVE':
            self.reassert_offboard_if_needed()
            if self.land_target is None:
                self.get_logger().warn('No landing target yet. Returning to ARUCO_SELECT.')
                self.state = 'ARUCO_SELECT'
                return

            x, y, z = self.land_target
            self.publish_trajectory_setpoint_odom(x=x, y=y, z=z)
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
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
            )
            self.state = 'COMPLETE'

        elif self.state == 'COMPLETE':
            self.get_logger().info('Mission complete.')

            if self.battery_at_mission_end is None and self.battery_percent is not None:
                self.battery_at_mission_end = self.battery_percent
                self.get_logger().info(
                    f'Captured battery_at_mission_end: {self.battery_at_mission_end:.4f}'
                )

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

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(msg)

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
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
        )
        self.get_logger().info('Arm command sent')
        self.armed = True

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info('Offboard mode command sent')


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