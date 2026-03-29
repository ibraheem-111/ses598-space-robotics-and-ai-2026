#!/usr/bin/env python3

import math
from typing import Iterable, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster


def quat_from_euler(roll: float, pitch: float, yaw: float) -> List[float]:
    """Return quaternion as [w, x, y, z]."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]


def quat_mul(q1: Iterable[float], q2: Iterable[float]) -> List[float]:
    """Hamilton product. Input/output are [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ]


def quat_conj(q: Iterable[float]) -> List[float]:
    w, x, y, z = q
    return [w, -x, -y, -z]


def quat_norm(q: Iterable[float]) -> float:
    w, x, y, z = q
    return math.sqrt(w * w + x * x + y * y + z * z)


def quat_normalize(q: Iterable[float]) -> List[float]:
    n = quat_norm(q)
    if n == 0.0:
        return [1.0, 0.0, 0.0, 0.0]
    return [v / n for v in q]


def quat_rotate(q: Iterable[float], v: Iterable[float]) -> List[float]:
    """Rotate vector v by quaternion q. q is [w, x, y, z]."""
    qn = quat_normalize(q)
    vq = [0.0, float(v[0]), float(v[1]), float(v[2])]
    out = quat_mul(quat_mul(qn, vq), quat_conj(qn))
    return [out[1], out[2], out[3]]


def quat_to_rotmat(q: Iterable[float]) -> List[List[float]]:
    """Return 3x3 active rotation matrix for quaternion [w, x, y, z]."""
    w, x, y, z = quat_normalize(q)

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ]


def transpose3(m: List[List[float]]) -> List[List[float]]:
    return [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]


def matmul3(a: List[List[float]], b: List[List[float]]) -> List[List[float]]:
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = (
                a[i][0] * b[0][j] +
                a[i][1] * b[1][j] +
                a[i][2] * b[2][j]
            )
    return out


def matvec3(a: List[List[float]], v: Iterable[float]) -> List[float]:
    x, y, z = v
    return [
        a[0][0] * x + a[0][1] * y + a[0][2] * z,
        a[1][0] * x + a[1][1] * y + a[1][2] * z,
        a[2][0] * x + a[2][1] * y + a[2][2] * z,
    ]


def diag3(d: Iterable[float]) -> List[List[float]]:
    d0, d1, d2 = d
    return [
        [float(d0), 0.0, 0.0],
        [0.0, float(d1), 0.0],
        [0.0, 0.0, float(d2)],
    ]


def cov_transform(cov: List[List[float]], r: List[List[float]]) -> List[List[float]]:
    """Return r * cov * r^T."""
    return matmul3(matmul3(r, cov), transpose3(r))


# Static frame transforms matching px4_ros_com:
# NED -> ENU: [N, E, D] -> [E, N, -D]
R_NED_TO_ENU = [
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, -1.0],
]

# FRD -> FLU: [F, R, D] -> [F, -R, -D]
R_FRD_TO_FLU = [
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
]

# These are the same static quaternions used by px4_ros_com:
# NED_ENU_Q = quaternion_from_euler(pi, 0, pi/2)
# AIRCRAFT_BASELINK_Q = quaternion_from_euler(pi, 0, 0)
Q_NED_TO_ENU = quat_from_euler(math.pi, 0.0, math.pi / 2.0)
Q_AIRCRAFT_TO_BASELINK = quat_from_euler(math.pi, 0.0, 0.0)


def ned_to_enu(v: Iterable[float]) -> List[float]:
    return matvec3(R_NED_TO_ENU, v)


def frd_to_flu(v: Iterable[float]) -> List[float]:
    return matvec3(R_FRD_TO_FLU, v)


def px4_to_ros_orientation(q_px4_wxyz: Iterable[float]) -> List[float]:
    """
    Reproduce px4_ros_com::frame_transforms::px4_to_ros_orientation():
      q_ros = NED_ENU_Q * q_px4 * AIRCRAFT_BASELINK_Q
    q_px4 is aircraft->NED, q_ros becomes base_link->ENU.
    """
    return quat_normalize(
        quat_mul(
            quat_mul(Q_NED_TO_ENU, q_px4_wxyz),
            Q_AIRCRAFT_TO_BASELINK,
        )
    )


def all_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(v)) for v in values)


class Px4VehicleOdometryToRosOdom(Node):
    def __init__(self) -> None:
        super().__init__("px4_vehicle_odometry_to_ros_odom")

        self.declare_parameter("px4_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("clock_topic", "/gz/clock")
        self.declare_parameter("odom_topic", "/rtabmap/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("unknown_angular_twist_variance", 1e6)

        self.px4_topic = self.get_parameter("px4_topic").value
        self.clock_topic = self.get_parameter("clock_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.unknown_ang_twist_var = float(
            self.get_parameter("unknown_angular_twist_variance").value
        )

        # PX4 uXRCE-DDS topics are commonly best-effort + volatile.
        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        pub_qos = QoSProfile(depth=10)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, pub_qos)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self._latest_clock = None

        self.clock_sub = self.create_subscription(
            Clock,
            self.clock_topic,
            self.clock_cb,
            sub_qos,
        )

        self.sub = self.create_subscription(
            VehicleOdometry,
            self.px4_topic,
            self.vehicle_odom_cb,
            sub_qos,
        )

        self._last_warn_ns = 0
        self.get_logger().info(
            f"Subscribing to {self.px4_topic} and {self.clock_topic}, publishing {self.odom_topic}, "
            f"TF {self.odom_frame} -> {self.base_frame}: {self.publish_tf}"
        )

    def clock_cb(self, msg: Clock) -> None:
        self._latest_clock = msg.clock

    def warn_throttle(self, text: str, period_sec: float = 2.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns > int(period_sec * 1e9):
            self.get_logger().warn(text)
            self._last_warn_ns = now_ns

    def vehicle_odom_cb(self, msg: VehicleOdometry) -> None:
        # This node keeps odom in a ROS-friendly fixed frame.
        # Support the common PX4 case shown by your topic echo: pose_frame=NED.
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.warn_throttle(
                f"Unsupported pose_frame={msg.pose_frame}. "
                f"This node currently supports only POSE_FRAME_NED."
            )
            return

        position_ned = [float(v) for v in msg.position]
        q_px4 = [float(v) for v in msg.q]

        if not all_finite(position_ned):
            self.warn_throttle("Skipping VehicleOdometry with invalid position.")
            return

        if not all_finite(q_px4) or quat_norm(q_px4) < 1e-9:
            self.warn_throttle("Skipping VehicleOdometry with invalid quaternion.")
            return

        if self._latest_clock is None:
            self.warn_throttle("No /clock received yet; skipping odometry publish.")
            return

        # Pose conversion
        position_enu = ned_to_enu(position_ned)
        q_ros = px4_to_ros_orientation(q_px4)  # base_link -> ENU

        # Twist conversion
        linear_body, linear_cov_body = self.convert_linear_velocity_and_cov(msg, q_ros)
        if linear_body is None or linear_cov_body is None:
            return

        angular_body = self.convert_angular_velocity(msg.angular_velocity)

        # Pose covariance
        pose_cov = [0.0] * 36
        pos_cov_world = self.convert_position_covariance(msg.position_variance)
        ori_cov_body = self.convert_orientation_covariance(msg.orientation_variance)

        pose_cov[0] = pos_cov_world[0][0]
        pose_cov[1] = pos_cov_world[0][1]
        pose_cov[2] = pos_cov_world[0][2]
        pose_cov[6] = pos_cov_world[1][0]
        pose_cov[7] = pos_cov_world[1][1]
        pose_cov[8] = pos_cov_world[1][2]
        pose_cov[12] = pos_cov_world[2][0]
        pose_cov[13] = pos_cov_world[2][1]
        pose_cov[14] = pos_cov_world[2][2]

        pose_cov[21] = ori_cov_body[0][0]
        pose_cov[22] = ori_cov_body[0][1]
        pose_cov[23] = ori_cov_body[0][2]
        pose_cov[27] = ori_cov_body[1][0]
        pose_cov[28] = ori_cov_body[1][1]
        pose_cov[29] = ori_cov_body[1][2]
        pose_cov[33] = ori_cov_body[2][0]
        pose_cov[34] = ori_cov_body[2][1]
        pose_cov[35] = ori_cov_body[2][2]

        # Twist covariance
        twist_cov = [0.0] * 36
        twist_cov[0] = linear_cov_body[0][0]
        twist_cov[1] = linear_cov_body[0][1]
        twist_cov[2] = linear_cov_body[0][2]
        twist_cov[6] = linear_cov_body[1][0]
        twist_cov[7] = linear_cov_body[1][1]
        twist_cov[8] = linear_cov_body[1][2]
        twist_cov[12] = linear_cov_body[2][0]
        twist_cov[13] = linear_cov_body[2][1]
        twist_cov[14] = linear_cov_body[2][2]

        # PX4 VehicleOdometry does not provide angular velocity variance.
        twist_cov[21] = self.unknown_ang_twist_var
        twist_cov[28] = self.unknown_ang_twist_var
        twist_cov[35] = self.unknown_ang_twist_var

        stamp = self._latest_clock

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = position_enu[0]
        odom.pose.pose.position.y = position_enu[1]
        odom.pose.pose.position.z = position_enu[2]

        # geometry_msgs uses x,y,z,w
        odom.pose.pose.orientation.x = q_ros[1]
        odom.pose.pose.orientation.y = q_ros[2]
        odom.pose.pose.orientation.z = q_ros[3]
        odom.pose.pose.orientation.w = q_ros[0]
        odom.pose.covariance = pose_cov

        odom.twist.twist.linear.x = linear_body[0]
        odom.twist.twist.linear.y = linear_body[1]
        odom.twist.twist.linear.z = linear_body[2]

        odom.twist.twist.angular.x = angular_body[0]
        odom.twist.twist.angular.y = angular_body[1]
        odom.twist.twist.angular.z = angular_body[2]
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame

            tf_msg.transform.translation.x = position_enu[0]
            tf_msg.transform.translation.y = position_enu[1]
            tf_msg.transform.translation.z = position_enu[2]

            tf_msg.transform.rotation.x = q_ros[1]
            tf_msg.transform.rotation.y = q_ros[2]
            tf_msg.transform.rotation.z = q_ros[3]
            tf_msg.transform.rotation.w = q_ros[0]

            self.tf_broadcaster.sendTransform(tf_msg)

    def convert_linear_velocity_and_cov(
        self,
        msg: VehicleOdometry,
        q_ros_base_to_enu: List[float],
    ) -> (Optional[List[float]], Optional[List[List[float]]]):
        vel = [float(v) for v in msg.velocity]
        vel_var = [float(v) for v in msg.velocity_variance]

        if not all_finite(vel) or not all_finite(vel_var):
            self.warn_throttle("Skipping VehicleOdometry with invalid velocity fields.")
            return None, None

        # nav_msgs/Odometry twist is in child_frame_id (base_link).
        # Convert to base_link frame.
        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED:
            vel_enu = ned_to_enu(vel)
            linear_body = quat_rotate(quat_conj(q_ros_base_to_enu), vel_enu)

            cov_ned = diag3(vel_var)
            cov_enu = cov_transform(cov_ned, R_NED_TO_ENU)

            r_base_to_enu = quat_to_rotmat(q_ros_base_to_enu)
            r_enu_to_base = transpose3(r_base_to_enu)
            cov_body = cov_transform(cov_enu, r_enu_to_base)
            return linear_body, cov_body

        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            linear_body = frd_to_flu(vel)

            cov_frd = diag3(vel_var)
            cov_body = cov_transform(cov_frd, R_FRD_TO_FLU)
            return linear_body, cov_body

        if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_FRD:
            self.warn_throttle(
                "Unsupported velocity_frame=VELOCITY_FRAME_FRD. "
                "This node currently supports only NED or BODY_FRD."
            )
            return None, None

        self.warn_throttle(
            f"Unsupported velocity_frame={msg.velocity_frame}. "
            "This node currently supports only NED or BODY_FRD."
        )
        return None, None

    def convert_angular_velocity(self, angular_velocity_frd: Iterable[float]) -> List[float]:
        ang = [float(v) for v in angular_velocity_frd]
        if not all_finite(ang):
            return [0.0, 0.0, 0.0]
        # PX4 angular_velocity is body-fixed FRD -> ROS base_link FLU.
        return frd_to_flu(ang)

    def convert_position_covariance(self, position_variance_ned: Iterable[float]) -> List[List[float]]:
        if not all_finite(position_variance_ned):
            return diag3([1e6, 1e6, 1e6])
        cov_ned = diag3(position_variance_ned)
        return cov_transform(cov_ned, R_NED_TO_ENU)

    def convert_orientation_covariance(self, orientation_variance_body_frd: Iterable[float]) -> List[List[float]]:
        if not all_finite(orientation_variance_body_frd):
            return diag3([1e6, 1e6, 1e6])
        cov_frd = diag3(orientation_variance_body_frd)
        return cov_transform(cov_frd, R_FRD_TO_FLU)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4VehicleOdometryToRosOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()