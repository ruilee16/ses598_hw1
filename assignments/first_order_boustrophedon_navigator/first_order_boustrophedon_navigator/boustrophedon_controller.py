#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import math
from collections import deque
from std_msgs.msg import Float64
from rcl_interfaces.msg import SetParametersResult
import matplotlib.pyplot as plt
from textwrap import fill


class BoustrophedonController(Node):
    def __init__(self):
        super().__init__('lawnmower_controller')
        self.get_logger().info("***** RUNNING UPDATED CONTROLLER: V4.1*****")

        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kp_linear', 1.0),
                ('Kd_linear', 0.1),
                ('Kp_angular', 1.0),
                ('Kd_angular', 0.1),
                ('spacing', 0.5)
            ]
        )
        
        # Get initial parameter values
        self.Kp_linear = self.get_parameter('Kp_linear').value
        self.Kd_linear = self.get_parameter('Kd_linear').value
        self.Kp_angular = self.get_parameter('Kp_angular').value
        self.Kd_angular = self.get_parameter('Kd_angular').value
        self.spacing = self.get_parameter('spacing').value

        self.get_logger().info(f"Model Parameters: <br>Kp_linear: {self.Kp_linear:.1f}, Kd_linear: {self.Kd_linear:.1f}, <br>Kp_angular: {self.Kd_angular:.1f}, Kd_angular: {self.Kd_angular:.1f}"
)
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Create publisher and subscriber
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # Lawnmower pattern parameters
        self.waypoints = self.generate_waypoints()
        self.current_waypoint = 0
        
        # Cross-track error calculation
        self.cross_track_errors = deque(maxlen=1000)  # Store last 1000 errors
        
        # Data for plots
        self.trajectory = []  # To store x, y positions
        self.velocities = []  # To store linear and angular velocities
        
        # State variables
        self.pose = Pose()
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.prev_time = self.get_clock().now()
        
        # Create control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Add publisher for cross-track error
        self.error_pub = self.create_publisher(
            Float64, 
            'cross_track_error', 
            10
        )
        
        self.get_logger().info('Lawnmower controller started')
        self.get_logger().info(f'Following waypoints: {self.waypoints}')

    def generate_waypoints(self):
        waypoints = []
        y = 8.0  # Start higher in the window
        
        while y >= 3.0:
            if len(waypoints) % 2 == 0:
                waypoints.append((2.0, y))
                waypoints.append((9.0, y))
            else:
                waypoints.append((9.0, y))
                waypoints.append((2.0, y))
            y -= self.spacing
        
        return waypoints

    def calculate_cross_track_error(self):
        if self.current_waypoint < 1:
            return 0.0

        start = np.array(self.waypoints[self.current_waypoint - 1])
        end = np.array(self.waypoints[self.current_waypoint])
        pos = np.array([self.pose.x, self.pose.y])

        path_vector = end - start
        path_length = np.linalg.norm(path_vector)
        if path_length < 1e-6:
            return np.linalg.norm(pos - start)

        path_unit = path_vector / path_length
        pos_vector = pos - start

        projection_length = np.dot(pos_vector, path_unit)
        projection_length = max(0, min(path_length, projection_length))
        projected_point = start + projection_length * path_unit

        error_vector = pos - projected_point
        error_sign = np.sign(np.cross(path_unit, error_vector / np.linalg.norm(error_vector)))
        error = np.linalg.norm(error_vector) * error_sign

        self.cross_track_errors.append(abs(error))

        error_msg = Float64()
        error_msg.data = error
        self.error_pub.publish(error_msg)

        return error

    def pose_callback(self, msg):
        self.pose = msg

    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_angle(self, x1, y1, x2, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def control_loop(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('Lawnmower pattern complete')
            if self.cross_track_errors:
                final_avg_error = sum(self.cross_track_errors) / len(self.cross_track_errors)
                self.get_logger().info(f'Final average cross-track error: {final_avg_error:.3f}')
            self.timer.cancel()
            self.plot_data()
            return

        cross_track_error = self.calculate_cross_track_error()

        target_x, target_y = self.waypoints[self.current_waypoint]
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9

        distance = self.get_distance(self.pose.x, self.pose.y, target_x, target_y)
        target_angle = self.get_angle(self.pose.x, self.pose.y, target_x, target_y)
        angular_error = target_angle - self.pose.theta

        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi

        linear_error_derivative = (distance - self.prev_linear_error) / dt
        angular_error_derivative = (angular_error - self.prev_angular_error) / dt

        linear_velocity = self.Kp_linear * distance + self.Kd_linear * linear_error_derivative
        angular_velocity = self.Kp_angular * angular_error + self.Kd_angular * angular_error_derivative

        vel_msg = Twist()
        vel_msg.linear.x = min(linear_velocity, 2.0)
        vel_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(vel_msg)

        self.trajectory.append((self.pose.x, self.pose.y))
        self.velocities.append((linear_velocity, angular_velocity))

        self.prev_linear_error = distance
        self.prev_angular_error = angular_error
        self.prev_time = current_time

        if distance < 0.1:
            self.current_waypoint += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint}')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'Kp_linear':
                self.Kp_linear = param.value
            elif param.name == 'Kd_linear':
                self.Kd_linear = param.value
            elif param.name == 'Kp_angular':
                self.Kp_angular = param.value
            elif param.name == 'Kd_angular':
                self.Kd_angular = param.value
            elif param.name == 'spacing':
                self.spacing = param.value
                self.waypoints = self.generate_waypoints()
        return SetParametersResult(successful=True)

    def plot_data_OG(self):
        trajectory = np.array(self.trajectory)
        velocities = np.array(self.velocities)

        # Plot Cross-Track Error
        plt.figure()
        plt.plot(self.cross_track_errors)
        plt.title("Cross-Track Error Over Time")
        plt.xlabel("Time Step")
        plt.ylabel("Error")
        plt.savefig("cross_track_error.png")

        # Plot Trajectory
        plt.figure()
        plt.plot(trajectory[:, 0], trajectory[:, 1], label="Trajectory")
        plt.scatter([wp[0] for wp in self.waypoints], [wp[1] for wp in self.waypoints], c='red', label="Waypoints")
        plt.title("Trajectory Plot")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.savefig("trajectory.png")

        # Plot Velocity Profiles
        plt.figure()
        plt.plot(velocities[:, 0], label="Linear Velocity")
        plt.plot(velocities[:, 1], label="Angular Velocity")
        plt.title("Velocity Profiles")
        plt.xlabel("Time Step")
        plt.ylabel("Velocity")
        plt.legend()
        plt.savefig("velocity_profiles.png")

        self.get_logger().info("Plots saved as PNG files.")

    def plot_data(self):
        """
        Create a single figure with 3 subplots:
        1) Cross-Track Error vs time step
        2) Trajectory with waypoints
        3) Velocity profiles (linear and angular)
        Also compute & annotate:
        - Average and Maximum Cross-Track Error
        - Smoothness of Motion (RMS of jerk computed from linear velocity)
            * If timestamps exist (self.timestamps), use real dt; otherwise assume dt=1.
        - Cornering performance: detect corner segments where |yaw_rate| > yaw_thresh,
            compute per-corner max lateral acceleration (a_lat = v * yaw_rate) and CTE during corner.
        Saves: "summary_plots.png", plus the individual PNGs.
        """
        self.get_logger().info("***** RUNNING UPDATED CONTROLLER: V3*****")


        # Convert to numpy arrays (safe if lists)
        trajectory = np.array(self.trajectory) if len(self.trajectory) > 0 else np.empty((0,2))
        velocities = np.array(self.velocities) if len(self.velocities) > 0 else np.empty((0,2))
        cte = np.array(self.cross_track_errors) if len(self.cross_track_errors) > 0 else np.empty((0,))

        # Time handling: if self.timestamps exists and matches length, use it; else use integer time steps
        if hasattr(self, 'timestamps') and len(getattr(self, 'timestamps')) == len(cte):
            times = np.array(self.timestamps)
            # convert to seconds if ROS Time msgs: assume already float seconds
        else:
            # fallback: uniform steps
            times = np.arange(len(cte))
            self.get_logger().warning("plot_data: timestamps not found or length mismatch — assuming uniform time step (dt=1).")

        # --- Compute average and max CTE ---
        if cte.size > 0:
            avg_cte = float(np.nanmean(np.abs(cte)))
            max_cte = float(np.nanmax(np.abs(cte)))
        else:
            avg_cte = float('nan')
            max_cte = float('nan')

        # --- Smoothness (RMS of jerk) ---
        # Use linear velocity (velocities[:,0]) to compute acceleration and jerk.
        # If timestamps available, compute time diffs; otherwise assume dt = 1.
        smooth_desc = ""
        smoothness_rms_jerk = float('nan')
        if velocities.shape[0] >= 3:
            linear_v = velocities[:, 0]
            # compute dt array
            if hasattr(self, 'timestamps') and len(getattr(self, 'timestamps')) == velocities.shape[0]:
                t_v = np.array(self.timestamps)
                dt = np.diff(t_v)
                # avoid zeros
                dt[dt == 0] = 1e-6
                acc = np.diff(linear_v) / dt        # len = n-1
                dt2 = dt[1:]
                jerk = np.diff(acc) / dt2           # len = n-2
                smooth_desc = "Smoothness = RMS(jerk) with real dt from timestamps."
            else:
                # assume uniform dt = 1
                acc = np.diff(linear_v)            # a[k] = v[k+1] - v[k]
                jerk = np.diff(acc)                # j[k] = a[k+1] - a[k]
                smooth_desc = "Smoothness = RMS(jerk) assuming uniform dt=1 (units: velocity units per step^3)."
            # safe compute
            if jerk.size > 0:
                smoothness_rms_jerk = float(np.sqrt(np.nanmean(jerk**2)))
            else:
                smoothness_rms_jerk = float('nan')
        else:
            smooth_desc = "Smoothness not computed (need >=3 velocity samples)."

        # --- Cornering performance ---
        # Use angular velocity as yaw_rate (velocities[:,1]), detect |yaw_rate| > yaw_thresh
        cornering_desc = ""
        corner_count = 0
        corner_metrics = []
        yaw_thresh = getattr(self, 'yaw_thresh', 0.1)  # rad/s default; you can set self.yaw_thresh elsewhere
        if velocities.shape[0] >= 1:
            yaw_rates = velocities[:, 1]
            linear_v = velocities[:, 0] if velocities.shape[1] >= 1 else np.zeros_like(yaw_rates)

            # detect corners where |yaw_rate| > yaw_thresh
            corners_bool = np.abs(yaw_rates) > yaw_thresh
            if np.any(corners_bool):
                # group contiguous indices
                idx = np.where(corners_bool)[0]
                groups = np.split(idx, np.where(np.diff(idx) != 1)[0] + 1)
                for g in groups:
                    start = g[0]; end = g[-1]
                    corner_count += 1
                    v_seg = linear_v[start:end+1]
                    yaw_seg = yaw_rates[start:end+1]
                    a_lat = np.abs(v_seg * yaw_seg)  # approx lateral accel (m/s^2)
                    max_a_lat = float(np.nanmax(a_lat)) if a_lat.size>0 else float('nan')
                    avg_a_lat = float(np.nanmean(a_lat)) if a_lat.size>0 else float('nan')

                    # compute max CTE within the same index range (if lengths align)
                    if cte.size == velocities.shape[0]:
                        corner_cte_seg = np.abs(cte[start:end+1])
                        max_cte_corner = float(np.nanmax(corner_cte_seg)) if corner_cte_seg.size>0 else float('nan')
                    else:
                        # fallback: try to find nearest CTE samples by index
                        # if cte shorter/longer, we'll attempt safe slicing by min length
                        minlen = min(cte.size, velocities.shape[0])
                        if minlen > 0:
                            s = max(0, int(start * minlen / velocities.shape[0]))
                            e = minlen-1 if end*minlen//velocities.shape[0] >= minlen else int(end * minlen / velocities.shape[0])
                            corner_cte_seg = np.abs(cte[s:e+1]) if e>=s else np.array([])
                            max_cte_corner = float(np.nanmax(corner_cte_seg)) if corner_cte_seg.size>0 else float('nan')
                        else:
                            max_cte_corner = float('nan')

                    corner_metrics.append({
                        "start_idx": int(start),
                        "end_idx": int(end),
                        "max_a_lat": max_a_lat,
                        "avg_a_lat": avg_a_lat,
                        "max_cte_corner": max_cte_corner
                    })
                cornering_desc = f"Corner detection threshold yaw_thresh={yaw_thresh} rad/s; detected {corner_count} corners."
            else:
                cornering_desc = f"No corners detected (|yaw_rate| <= {yaw_thresh})."
        else:
            cornering_desc = "Cornering not computed (no velocity samples)."

        # --- Plotting: single figure with 3 subplots ---
        fig, axs = plt.subplots(2, 2, figsize=(10, 14), constrained_layout=True)

        # 1) CTE plot
        axs[0, 0].plot(times if times.size>0 else np.arange(len(cte)), cte, label='Cross-Track Error')
        axs[0, 0].set_title("Cross-Track Error Over Time")
        axs[0, 0].set_xlabel("Time (s) / Time step")
        axs[0, 0].set_ylabel("CTE (m)")
        axs[0, 0].grid(True)

        # 2) Trajectory plot
        if trajectory.size > 0:
            axs[0, 1].plot(trajectory[:, 0], trajectory[:, 1], label="Trajectory")
            if hasattr(self, 'waypoints') and len(self.waypoints) > 0:
                wp_x = [wp[0] for wp in self.waypoints]
                wp_y = [wp[1] for wp in self.waypoints]
                axs[0, 1].scatter(wp_x, wp_y, c='red', label='Waypoints', s=30, zorder=3)
        axs[0, 1].set_title("Trajectory Plot")
        axs[0, 1].set_xlabel("X (m)")
        axs[0, 1].set_ylabel("Y (m)")
        axs[0, 1].legend()
        axs[0, 1].axis('equal')
        axs[0, 1].grid(True)

        # 3) Velocity profiles
        if velocities.size > 0:
            axs[1, 0].plot(velocities[:, 0], label="Linear Velocity (v)")
            axs[1, 0].plot(velocities[:, 1], label="Angular Velocity (yaw_rate)")
        axs[1, 0].set_title("Velocity Profiles")
        axs[1, 0].set_xlabel("Time step")
        axs[1, 0].set_ylabel("Velocity (m/s or rad/s)")
        axs[1, 0].legend()
        axs[1, 0].grid(True)

        # --- Prepare summary text (metrics) for annotation ---
        # Build corner summary string (brief)
        corner_summary_lines = []
        for i, cm in enumerate(corner_metrics[:5]):  # show up to first 5 corners
            corner_summary_lines.append(f"C{i+1}: max_a_lat={cm['max_a_lat']:.3f}, max_cte={cm['max_cte_corner']:.3f}")

        corner_summary = "\n".join(corner_summary_lines) if corner_summary_lines else "No corner metrics"

        summary_text = (
            f"*****SUMMARY*****<br>"
            f"Model Parameters: <br>Kp_linear: {self.Kp_linear:.1f}, Kd_linear: {self.Kd_linear:.1f}, <br>Kp_angular: {self.Kd_angular:.1f}, Kd_angular: {self.Kd_angular:.1f}"
            f"Average CTE = {avg_cte:.4f} m<br>"
            f"Maximum CTE = {max_cte:.4f} m<br>"
            f"Smoothness = {smoothness_rms_jerk:.6g}<br>"
            f"{'(computed from linear v; ' + smooth_desc + ')'}<br>"
            f"Corner count = {corner_count}<br>"
            f"{cornering_desc}<br>"
            f"{corner_summary}<br>"
            "Notes:<br>"
            "- Smoothness metric: RMS of turtle (d^2 v / dt^2). Lower is smoother.<br>"
            "- Cornering: a_lat ~ v * yaw_rate (approx). Lower peak a_lat and lower corner CTE are better."
        )

        # Place summary textbox on the figure (right side)
        axs[1, 1].text(0.02, 0.02, fill(summary_text, width=60), fontsize=9, va='bottom', ha='left',
                bbox=dict(facecolor='white', alpha=0.9, edgecolor='black'))

        # Save files
        try:
            fig.savefig("summary_plots.png", dpi=200)
            # also save individual images for compatibility
            # CTE
            plt.figure()
            plt.plot(times if times.size>0 else np.arange(len(cte)), cte)
            plt.title("Cross-Track Error Over Time")
            plt.xlabel("Time Step")
            plt.ylabel("Error")
            plt.grid(True)
            plt.savefig("cross_track_error.png", dpi=200)
            plt.close()

            # Trajectory
            if trajectory.size>0:
                plt.figure()
                plt.plot(trajectory[:,0], trajectory[:,1], label="Trajectory")
                if hasattr(self, 'waypoints') and len(self.waypoints) > 0:
                    plt.scatter([wp[0] for wp in self.waypoints], [wp[1] for wp in self.waypoints], c='red', label='Waypoints')
                plt.title("Trajectory Plot")
                plt.xlabel("X")
                plt.ylabel("Y")
                plt.legend()
                plt.axis('equal')
                plt.grid(True)
                plt.savefig("trajectory.png", dpi=200)
                plt.close()

            # Velocity profiles
            if velocities.size>0:
                plt.figure()
                plt.plot(velocities[:,0], label="Linear Velocity")
                plt.plot(velocities[:,1], label="Angular Velocity")
                plt.title("Velocity Profiles")
                plt.xlabel("Time Step")
                plt.ylabel("Velocity")
                plt.legend()
                plt.grid(True)
                plt.savefig("velocity_profiles.png", dpi=200)
                plt.close()

            self.get_logger().info("Plots saved as summary_plots.png and individual PNGs.")
        except Exception as e:
            self.get_logger().error(f"Failed to save plots: {e}")

        # --- Log metrics as well (for programmatic checks) ---
        try:
            self.get_logger().info(f"avg_cte={avg_cte:.6g}, max_cte={max_cte:.6g}, smoothness_rms_jerk={smoothness_rms_jerk:.6g}, corner_count={corner_count}")
        except Exception:
            pass

        # Optionally return metrics if you want to use them programmatically
        return {
            "avg_cte": avg_cte,
            "max_cte": max_cte,
            "smoothness_rms_jerk": smoothness_rms_jerk,
            "corner_count": corner_count,
            "corner_metrics": corner_metrics
        }




def main(args=None):
    rclpy.init(args=args)
    controller = BoustrophedonController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
