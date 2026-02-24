#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
from collections import deque
import csv
import os
from datetime import datetime

class CartPoleLQRController(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr_controller')
        
        # System parameters
        self.M = 1.0  # Mass of cart (kg)
        self.m = 1.0  # Mass of pole (kg)
        self.L = 1.0  # Length of pole (m)
        self.g = 9.81  # Gravity (m/s^2)
        
        # State space matrices
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.m * self.g) / self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M + self.m) * self.g) / (self.M * self.L), 0]
        ])
        
        self.B = np.array([
            [0],
            [1/self.M],
            [0],
            [-1/(self.M * self.L)]
        ])
        
        # LQR cost matrices
        self.Q = np.diag([10.5, 10.5, 12.0, 12.0])  # State cost
        self.R = np.array([[0.4]])  # Control cost
        
        # Compute LQR gain matrix
        self.K = self.compute_lqr_gain()
        self.get_logger().info(f'LQR Gain Matrix: {self.K}')
        
        # Initialize state estimate
        self.x = np.zeros((4, 1))
        self.state_initialized = False
        self.last_control = 0.0
        self.control_count = 0
        
        # Data storage for plotting
        self.time_steps = deque()
        self.cart_positions = deque()
        self.pole_angles = deque()
        self.control_forces = deque()
        self.earthquake_forces = deque()
        self.start_time = None
        self.termination_grace_period = 1.0
        self._warned_pre_state_earthquake = False

        log_dir = os.path.join(os.path.expanduser('~'), 'ws_ses')
        if not os.path.isdir(log_dir):
            log_dir = os.getcwd()
        self.results_log_path = os.path.join(log_dir, 'lqr_experiments.csv')
        
        # Create publishers and subscribers
        self.cart_cmd_pub = self.create_publisher(Float64, '/model/cart_pole/joint/cart_to_base/cmd_force', 10)
        
        if self.cart_cmd_pub:
            self.get_logger().info('Force command publisher created successfully')
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.earthquake_sub = self.create_subscription(Float64, '/earthquake_force', self.earthquake_callback, 10)
        
        # Control loop timer
        self.timer = self.create_timer(0.01, self.control_loop)

        self.MAX_SIMULATION_TIME = 120.0  # Set to desired duration
        
        self.get_logger().info('Cart-Pole LQR Controller initialized')
    
    def compute_lqr_gain(self):
        """Compute the LQR gain matrix K."""
        P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ self.B.T @ P
        return K
    
    def joint_state_callback(self, msg):
        """Update state estimate from joint states."""
        try:
            cart_idx = msg.name.index('cart_to_base')
            pole_idx = msg.name.index('pole_joint')
            
            self.x = np.array([
                [msg.position[cart_idx]],
                [msg.velocity[cart_idx]],
                [msg.position[pole_idx]],
                [msg.velocity[pole_idx]]
            ])
            
            if not self.state_initialized:
                self.get_logger().info(f'Initial state: cart_pos={msg.position[cart_idx]:.3f}, cart_vel={msg.velocity[cart_idx]:.3f}, pole_angle={msg.position[pole_idx]:.3f}, pole_vel={msg.velocity[pole_idx]:.3f}')
                self.state_initialized = True
                self.start_time = self.get_clock().now().nanoseconds / 1e9
                
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to process joint states: {e}, msg={msg.name}')

    def earthquake_callback(self, msg):
        """Store earthquake force values."""
        if self.state_initialized:
            self.earthquake_forces.append(msg.data)
        elif not self._warned_pre_state_earthquake:
            self.get_logger().warn("Received earthquake force before state was initialized.")
            self._warned_pre_state_earthquake = True

    def _compute_metrics(self):
        """Compute performance metrics from collected trajectories."""
        total_duration = self.time_steps[-1] if self.time_steps else 0.0

        stable_duration = total_duration
        time_values = list(self.time_steps)
        cart_values = list(self.cart_positions)
        pole_values_deg = list(self.pole_angles)

        for index, (cart_position, pole_angle_deg) in enumerate(zip(cart_values, pole_values_deg)):
            violated = abs(cart_position) > 2.5 or abs(pole_angle_deg) > 45.0
            if violated:
                stable_duration = time_values[index - 1] if index > 0 else 0.0
                break

        max_cart_displacement = max(map(abs, self.cart_positions), default=0.0)
        max_pole_deviation = max(map(abs, self.pole_angles), default=0.0)
        avg_control_effort = np.mean(np.abs(self.control_forces)) if self.control_forces else 0.0
        peak_control_effort = max(map(abs, self.control_forces), default=0.0)

        base_score = max(0, 10 - (max_cart_displacement * 2) - (max_pole_deviation / 5) - (avg_control_effort / 20))
        stable_ratio = stable_duration / total_duration if total_duration > 0.0 else 0.0
        stability_score = base_score * stable_ratio

        return {
            'total_duration': total_duration,
            'stable_duration': stable_duration,
            'stable_ratio': stable_ratio,
            'max_cart_displacement': max_cart_displacement,
            'max_pole_deviation': max_pole_deviation,
            'avg_control_effort': float(avg_control_effort),
            'peak_control_effort': float(peak_control_effort),
            'stability_score': float(stability_score),
        }

    def _append_experiment_log(self, metrics):
        """Append current parameters and outcomes to CSV log."""
        fieldnames = [
            'timestamp', 'q_x', 'q_x_dot', 'q_theta', 'q_theta_dot', 'r',
            'stable_duration_s', 'total_runtime_s', 'stable_ratio',
            'max_cart_displacement_m', 'max_pole_deviation_deg',
            'avg_control_effort_n', 'peak_control_effort_n',
            'stability_score_0_10', 'termination_grace_s', 'max_sim_time_s'
        ]

        row = {
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'q_x': float(self.Q[0, 0]),
            'q_x_dot': float(self.Q[1, 1]),
            'q_theta': float(self.Q[2, 2]),
            'q_theta_dot': float(self.Q[3, 3]),
            'r': float(self.R[0, 0]),
            'stable_duration_s': metrics['stable_duration'],
            'total_runtime_s': metrics['total_duration'],
            'stable_ratio': metrics['stable_ratio'],
            'max_cart_displacement_m': metrics['max_cart_displacement'],
            'max_pole_deviation_deg': metrics['max_pole_deviation'],
            'avg_control_effort_n': metrics['avg_control_effort'],
            'peak_control_effort_n': metrics['peak_control_effort'],
            'stability_score_0_10': metrics['stability_score'],
            'termination_grace_s': self.termination_grace_period,
            'max_sim_time_s': self.MAX_SIMULATION_TIME,
        }

        file_exists = os.path.exists(self.results_log_path)
        with open(self.results_log_path, 'a', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

    def print_metrics(self):
        """Prints performance metrics after simulation ends."""
        metrics = self._compute_metrics()


        self.get_logger().info(f"Q values: {self.Q.diagonal()}, R values: {self.R}")
        self.get_logger().info(f"Duration of stable operation: {metrics['stable_duration']:.2f} s")
        self.get_logger().info(f"Total simulation runtime: {metrics['total_duration']:.2f} s")
        self.get_logger().info(f"Stable runtime ratio: {metrics['stable_ratio']:.3f}")
        self.get_logger().info(f"Maximum cart displacement: {metrics['max_cart_displacement']:.3f} m")
        self.get_logger().info(f"Maximum pendulum angle deviation: {metrics['max_pole_deviation']:.3f}°")
        self.get_logger().info(f"Average control effort: {metrics['avg_control_effort']:.3f} N")
        self.get_logger().info(f"Peak control effort: {metrics['peak_control_effort']:.3f} N")
        self.get_logger().info(f"Stability score: {metrics['stability_score']:.2f}/10")

        self._append_experiment_log(metrics)
        self.get_logger().info(f"Experiment logged to: {self.results_log_path}")



    def control_loop(self):
        """Compute and apply LQR control."""
        try:
            if not self.state_initialized:
                self.get_logger().warn('State not initialized yet')
                return

            u = -self.K @ self.x
            force = float(u[0])
            
            msg = Float64()
            msg.data = force
            self.cart_cmd_pub.publish(msg)
            
            self.last_control = force
            self.control_count += 1
            
            # Ensure time steps are synchronized
            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            self.time_steps.append(current_time)
            self.cart_positions.append(self.x[0, 0])
            self.pole_angles.append(np.degrees(self.x[2, 0]))
            self.control_forces.append(force)

            # Ensure earthquake force logging matches other data dimensions
            if len(self.earthquake_forces) < len(self.time_steps):
                self.earthquake_forces.append(self.earthquake_forces[-1] if self.earthquake_forces else 0.0)

            # **Termination Conditions**
            unstable = abs(self.x[0, 0]) > 2.5 or abs(self.x[2, 0]) > np.radians(45)
            timed_out = current_time >= self.MAX_SIMULATION_TIME

            if timed_out or (current_time >= self.termination_grace_period and unstable):
                self.get_logger().warn(f"Simulation ended: cart_x={self.x[0, 0]:.2f}m, pole_angle={np.degrees(self.x[2, 0]):.2f}°, duration={current_time:.2f}s")
                self.print_metrics()
                self.plot_results()
                rclpy.shutdown()
                return

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')

    def plot_results(self):
        """Generate plots for analysis."""
        plt.figure(figsize=(12, 10))
        
        plt.subplot(2, 2, 1)
        plt.plot(self.time_steps, self.cart_positions, label='Cart Position (m)', color='b')
        plt.xlabel('Time (s)')
        plt.ylabel('Cart Position (m)')
        plt.legend()
        
        plt.subplot(2, 2, 2)
        plt.plot(self.time_steps, self.pole_angles, label='Pole Angle (°)', color='r')
        plt.xlabel('Time (s)')
        plt.ylabel('Pole Angle (°)')
        plt.legend()
        
        plt.subplot(2, 2, 3)
        plt.plot(self.time_steps, self.earthquake_forces, label='Earthquake Force (N)', color='g')
        plt.xlabel('Time (s)')
        plt.ylabel('Earthquake Force (N)')
        plt.legend()
        
        plt.subplot(2, 2, 4)
        plt.plot(self.time_steps, self.control_forces, label='Control Force (N)', color='m')  # Changed 'p' to 'm' (magenta)
        plt.xlabel('Time (s)')
        plt.ylabel('Control Force (N)')
        plt.legend()

        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    controller = CartPoleLQRController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
