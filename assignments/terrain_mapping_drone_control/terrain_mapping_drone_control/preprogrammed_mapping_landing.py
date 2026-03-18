#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry


class PreprogrammedMappingLanding(Node):
    """Fly a fixed mapping trajectory and then command landing on the front cylinder."""

    def __init__(self):
        super().__init__("preprogrammed_mapping_landing")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.odom_callback, qos_profile
        )

        self.position = [0.0, 0.0, 0.0]
        self.have_odom = False

        self.offboard_setpoint_counter = 0
        self.state = "INIT"
        self.state_enter_time = time.time()

        self.takeoff_height = -5.0
        self.waypoint_tolerance = 0.6
        self.waypoint_hold_time_sec = 2.0

        # NED-frame mapping path around the cylinders and nearby terrain region.
        self.mapping_waypoints = [
            (0.0, 0.0, self.takeoff_height),
            (5.0, -3.0, self.takeoff_height),
            (8.0, 0.0, self.takeoff_height),
            (5.0, 3.0, self.takeoff_height),
            (0.0, 6.0, self.takeoff_height),
            (-5.0, 3.0, self.takeoff_height),
            (-8.0, 0.0, self.takeoff_height),
            (-5.0, -3.0, self.takeoff_height),
            (0.0, -6.0, self.takeoff_height),
            (5.0, 0.0, self.takeoff_height),
            (-5.0, 0.0, self.takeoff_height),
            (0.0, 0.0, self.takeoff_height),
        ]
        self.waypoint_index = 0
        self.waypoint_reached_since = None

        # Front cylinder spawn is near (5, 0). We descend and then issue NAV_LAND.
        self.landing_xy = (5.0, 0.0)
        self.preland_height = -1.5
        self.land_command_sent = False

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Preprogrammed mapping+landing node started")

    def odom_callback(self, msg: VehicleOdometry):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]
        self.have_odom = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def engage_offboard_mode(self):
        # VEHICLE_CMD_DO_SET_MODE with custom mode 6 = offboard.
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def command_land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("LAND command sent")

    def _distance_to(self, target):
        dx = self.position[0] - target[0]
        dy = self.position[1] - target[1]
        dz = self.position[2] - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _set_state(self, new_state: str):
        self.state = new_state
        self.state_enter_time = time.time()
        self.get_logger().info(f"State -> {new_state}")

    def control_loop(self):
        self.publish_offboard_control_mode()

        # Always stream a valid setpoint before mode switch.
        if self.state in ("INIT", "TAKEOFF"):
            self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)

        self.offboard_setpoint_counter += 1

        if not self.have_odom:
            if self.offboard_setpoint_counter % 20 == 0:
                self.get_logger().warn("Waiting for /fmu/out/vehicle_odometry...")
            return

        if self.state == "INIT":
            # PX4 requires setpoint stream before entering offboard.
            if self.offboard_setpoint_counter >= 15:
                self.engage_offboard_mode()
                self.arm()
                self._set_state("TAKEOFF")
            return

        if self.state == "TAKEOFF":
            target = (0.0, 0.0, self.takeoff_height)
            self.publish_trajectory_setpoint(*target)
            if self._distance_to(target) < self.waypoint_tolerance:
                self._set_state("MAP_TRAJECTORY")
            return

        if self.state == "MAP_TRAJECTORY":
            target = self.mapping_waypoints[self.waypoint_index]
            self.publish_trajectory_setpoint(*target)

            if self._distance_to(target) < self.waypoint_tolerance:
                if self.waypoint_reached_since is None:
                    self.waypoint_reached_since = time.time()
                elif time.time() - self.waypoint_reached_since >= self.waypoint_hold_time_sec:
                    self.waypoint_index += 1
                    self.waypoint_reached_since = None
                    if self.waypoint_index >= len(self.mapping_waypoints):
                        self._set_state("PRELAND_ALIGN")
            else:
                self.waypoint_reached_since = None
            return

        if self.state == "PRELAND_ALIGN":
            target = (self.landing_xy[0], self.landing_xy[1], self.preland_height)
            self.publish_trajectory_setpoint(*target)
            if self._distance_to(target) < self.waypoint_tolerance:
                self._set_state("LAND")
            return

        if self.state == "LAND":
            if not self.land_command_sent:
                self.command_land()
                self.land_command_sent = True
                self.state_enter_time = time.time()
            # Give autopilot time to land. Then disarm as a safety fallback.
            if time.time() - self.state_enter_time > 20.0:
                self.disarm()
                self._set_state("COMPLETE")
            return

        if self.state == "COMPLETE":
            # Keep node alive but quiet.
            return


def main(args=None):
    rclpy.init(args=args)
    node = PreprogrammedMappingLanding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
