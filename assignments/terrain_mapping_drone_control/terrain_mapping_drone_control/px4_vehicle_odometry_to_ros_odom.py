#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_multiply


def all_finite(values) -> bool:
    return all(math.isfinite(float(v)) for v in values)


class Px4VehicleOdometryRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("px4_vehicle_odometry_republisher")

        self.declare_parameter("px4_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("clock_topic", "/gz/clock")
        self.declare_parameter("odom_topic", "/rtabmap/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.px4_topic = str(self.get_parameter("px4_topic").value)
        self.clock_topic = str(self.get_parameter("clock_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(depth=10)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, pub_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._latest_clock = None
        self._last_warn_ns = 0

        self.clock_sub = self.create_subscription(
            Clock,
            self.clock_topic,
            self.clock_cb,
            sub_qos,
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            self.px4_topic,
            self.vehicle_odom_cb,
            sub_qos,
        )

        self.get_logger().info(
            f"Republishing {self.px4_topic} to {self.odom_topic} and TF "
            f"{self.odom_frame} -> {self.base_frame}"
        )

    def clock_cb(self, msg: Clock) -> None:
        self._latest_clock = msg.clock

    def warn_throttle(self, text: str, period_sec: float = 2.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns > int(period_sec * 1e9):
            self.get_logger().warn(text)
            self._last_warn_ns = now_ns

    def vehicle_odom_cb(self, msg: VehicleOdometry) -> None:
        if self._latest_clock is None:
            self.warn_throttle("No /clock received yet; skipping publish.")
            return

        position = [float(v) for v in msg.position]
        q = [float(v) for v in msg.q]
        velocity = [float(v) for v in msg.velocity]
        angular_velocity = [float(v) for v in msg.angular_velocity]
        position_variance = [float(v) for v in msg.position_variance]
        orientation_variance = [float(v) for v in msg.orientation_variance]
        velocity_variance = [float(v) for v in msg.velocity_variance]

        if not all_finite(position):
            self.warn_throttle("Skipping message with invalid position.")
            return

        if not all_finite(q):
            self.warn_throttle("Skipping message with invalid quaternion.")
            return

        if not all_finite(velocity):
            self.warn_throttle("Skipping message with invalid velocity.")
            return

        if not all_finite(angular_velocity):
            self.warn_throttle("Skipping message with invalid angular velocity.")
            return

        stamp = self._latest_clock

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position: NED -> ENU
        odom.pose.pose.position.x = position[1]
        odom.pose.pose.position.y = position[0]
        odom.pose.pose.position.z = -position[2]

        # Quaternion: PX4 [w,x,y,z] (NED/FRD) -> ROS [x,y,z,w] (ENU/FLU)
        q_xyzw = [q[1], q[2], q[3], q[0]]
        q_ned_to_enu = [math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]
        q_frd_to_flu = [1.0, 0.0, 0.0, 0.0]
        q_enu_flu = quaternion_multiply(quaternion_multiply(q_ned_to_enu, q_xyzw), q_frd_to_flu)

        q_norm = math.sqrt(sum(c * c for c in q_enu_flu))
        if q_norm <= 1e-12:
            self.warn_throttle("Skipping message with near-zero quaternion norm.")
            return
        q_enu_flu = [c / q_norm for c in q_enu_flu]

        odom.pose.pose.orientation.x = q_enu_flu[0]
        odom.pose.pose.orientation.y = q_enu_flu[1]
        odom.pose.pose.orientation.z = q_enu_flu[2]
        odom.pose.pose.orientation.w = q_enu_flu[3]

        # Linear velocity: NED -> ENU
        odom.twist.twist.linear.x = velocity[1]
        odom.twist.twist.linear.y = velocity[0]
        odom.twist.twist.linear.z = -velocity[2]

        # Angular velocity is body FRD in PX4 -> body FLU in ROS
        odom.twist.twist.angular.x = angular_velocity[0]
        odom.twist.twist.angular.y = -angular_velocity[1]
        odom.twist.twist.angular.z = -angular_velocity[2]

        # Minimal covariance passthrough on diagonals only
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36

        if all_finite(position_variance):
            pose_cov[0] = position_variance[1]
            pose_cov[7] = position_variance[0]
            pose_cov[14] = position_variance[2]

        if all_finite(orientation_variance):
            pose_cov[21] = orientation_variance[0]
            pose_cov[28] = orientation_variance[1]
            pose_cov[35] = orientation_variance[2]

        if all_finite(velocity_variance):
            twist_cov[0] = velocity_variance[1]
            twist_cov[7] = velocity_variance[0]
            twist_cov[14] = velocity_variance[2]

        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.base_frame

        tf_msg.transform.translation.x = odom.pose.pose.position.x
        tf_msg.transform.translation.y = odom.pose.pose.position.y
        tf_msg.transform.translation.z = odom.pose.pose.position.z

        tf_msg.transform.rotation.x = q_enu_flu[0]
        tf_msg.transform.rotation.y = q_enu_flu[1]
        tf_msg.transform.rotation.z = q_enu_flu[2]
        tf_msg.transform.rotation.w = q_enu_flu[3]


        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4VehicleOdometryRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()