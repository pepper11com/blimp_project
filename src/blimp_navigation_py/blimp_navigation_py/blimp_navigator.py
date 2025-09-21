#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
import math
import numpy as np
import asyncio
import tf2_ros
from tf2_ros import TransformException

from blimp_navigation_py.pid import PID
from blimp_navigation_py.blimp_controller import BlimpController
from blimp_navigation.srv import CheckCollision

class BlimpNavigatorNode(Node):
    def __init__(self):
        super().__init__('blimp_navigator_node')

        self.DEBUG_MODE = True

        # --- Parameters for tuning ---
        self.declare_parameter('altitude_kp', 150.0)
        self.declare_parameter('altitude_ki', 10.0)
        self.declare_parameter('altitude_kd', 20.0)
        self.declare_parameter('heading_kp', 0.3)
        self.declare_parameter('heading_ki', 0.05)
        self.declare_parameter('heading_kd', 0.1)
        self.declare_parameter('forward_kp', 20.0)
        self.declare_parameter('forward_ki', 2.0)
        self.declare_parameter('forward_kd', 5.0)

        # --- TF2 listener for getting the correct pose from the map frame ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target_frame = 'map'
        self.source_frame = 'camera_link'

        self.current_pose = None
        self.current_heading_rad = 0.0
        self.goal_pose = None
        self.obstacle_detected = False

        try:
            self.controller = BlimpController()
            self.get_logger().info("Successfully connected to the flight controller.")
        except Exception as e:
            self.get_logger().fatal(f"FATAL: Failed to connect to flight controller: {e}")
            self.controller = None

        # --- PID controllers ---
        self.altitude_pid = PID(
            Kp=self.get_parameter('altitude_kp').value, Ki=self.get_parameter('altitude_ki').value, Kd=self.get_parameter('altitude_kd').value,
            setpoint=0, output_limits=(-500, 500)
        )
        self.heading_pid = PID(
            Kp=self.get_parameter('heading_kp').value, Ki=self.get_parameter('heading_ki').value, Kd=self.get_parameter('heading_kd').value,
            setpoint=0, output_limits=(-50, 50)
        )
        self.forward_pid = PID(
            Kp=self.get_parameter('forward_kp').value, Ki=self.get_parameter('forward_ki').value, Kd=self.get_parameter('forward_kd').value,
            setpoint=0, output_limits=(0, 20)
        )
        self.get_logger().info("PID controllers initialized.")

        # --- ROS2 communications ---
        self.goal_subscriber = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.collision_client = self.create_client(CheckCollision, '/check_collision')
        if not self.collision_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Collision check service not available!')

        # Consolidated status publisher
        if self.DEBUG_MODE:
            self.status_publisher = self.create_publisher(String, '/blimp_status', 10)
            self.last_debug_time = self.get_clock().now()

        # --- Main loop ---
        self.navigation_task = self.create_timer(0.1, self.navigate_loop_wrapper)
        self.get_logger().info("Blimp PID Navigator Node has been started.")

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg.pose
        self.get_logger().info(f"New goal received: Pos(x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f})")
        self.altitude_pid.set_setpoint(self.goal_pose.position.z)

    def get_current_pose_from_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rclpy.time.Time())
            self.current_pose = PoseStamped()
            self.current_pose.header.stamp = trans.header.stamp
            self.current_pose.header.frame_id = self.target_frame
            self.current_pose.pose.position = trans.transform.translation
            self.current_pose.pose.orientation = trans.transform.rotation
            q = self.current_pose.pose.orientation
            self.current_heading_rad = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return True
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {self.source_frame} to {self.target_frame}: {ex}', throttle_duration_sec=2.0)
            return False

    async def check_for_obstacles(self):
        if self.current_pose is None or not self.collision_client.service_is_ready():
            return False
            
        request = CheckCollision.Request()
        points_to_check = []
        pos = self.current_pose.pose.position
        forward_vector = np.array([math.cos(self.current_heading_rad), math.sin(self.current_heading_rad), 0])
        # Check points in a line in front of the blimp
        for step in np.linspace(0.3, 1.5, num=10):
            check_pos = np.array([pos.x, pos.y, pos.z]) + forward_vector * step
            points_to_check.append(Point(x=check_pos[0], y=check_pos[1], z=check_pos[2]))
        
        request.points_to_check = points_to_check
        future = self.collision_client.call_async(request)
        try:
            response = await asyncio.wait_for(future, timeout=0.2)
            return response.is_occupied
        except asyncio.TimeoutError:
            return False # Fail safe

    def navigate_loop_wrapper(self):
        asyncio.run(self.navigate_loop())

    async def navigate_loop(self):
        if not self.controller: return

        if not self.get_current_pose_from_tf():
            self.get_logger().warn("Waiting for TF transform from map to camera_link...", throttle_duration_sec=5.0)
            return

        # Check for obstacles once per loop
        self.obstacle_detected = await self.check_for_obstacles()

        if not self.goal_pose:
            self.controller.set_motor_pct(0, 0)
            self.controller.set_motor_pct(1, 0)
            # We still publish debug info even without a goal
        else:
            # --- Main navigation logic ---
            pos = self.current_pose.pose.position
            goal_pos = self.goal_pose.position
            vector_to_goal_2d = np.array([goal_pos.x - pos.x, goal_pos.y - pos.y])
            distance_to_goal_2d = np.linalg.norm(vector_to_goal_2d)

            if distance_to_goal_2d < 0.5:
                self.get_logger().info("Goal reached!")
                self.goal_pose = None
                self.controller.set_motor_pct(0, 0); self.controller.set_motor_pct(1, 0)
            else:
                # --- PID calculations ---
                altitude_output = self.altitude_pid.update(pos.z)
                desired_heading_rad = math.atan2(vector_to_goal_2d[1], vector_to_goal_2d[0])
                heading_error_rad = desired_heading_rad - self.current_heading_rad
                if heading_error_rad > math.pi: heading_error_rad -= 2 * math.pi
                if heading_error_rad < -math.pi: heading_error_rad += 2 * math.pi
                
                turn_output = self.heading_pid.update(math.degrees(heading_error_rad))
                self.forward_pid.set_setpoint(0)
                forward_output = self.forward_pid.update(-distance_to_goal_2d)
                
                # --- Control logic overrides ---
                if abs(math.degrees(heading_error_rad)) > 25.0:
                    forward_output = 0 # Prioritize turning
                
                if self.obstacle_detected:
                    forward_output = 0; turn_output = -40.0 # Evasive action

                # --- Motor command calculation ---
                servo_deflection = int(altitude_output) if altitude_output is not None else 0
                turn_val = turn_output if turn_output is not None else 0
                fwd_val = forward_output if forward_output is not None else 0
                
                left_motor = fwd_val + turn_val
                right_motor = fwd_val - turn_val
                
                left_motor_final = np.clip(left_motor, 0, 20)
                right_motor_final = np.clip(right_motor, 0, 20)

                # --- Send commands to hardware ---
                self.controller.set_servo_us(2, 1500 + servo_deflection)
                self.controller.set_servo_us(3, 1500 + servo_deflection)
                self.controller.set_motor_pct(0, left_motor_final)
                self.controller.set_motor_pct(1, right_motor_final)

        # --- Consolidated and throttled debug publisher ---
        if self.DEBUG_MODE:
            now = self.get_clock().now()
            if (now - self.last_debug_time).nanoseconds / 1e9 >= 1.5:
                self.last_debug_time = now
                
                status_msg = String()
                pos = self.current_pose.pose.position
                
                # Build the multi-line status string
                status_report = f"""
                =================== BLIMP STATUS ===================
                - OBSTACLE AHEAD: {self.obstacle_detected}
                ----------------------------------------------------
                |         |      X     |      Y     |      Z     |
                | Current | {pos.x:10.2f} | {pos.y:10.2f} | {pos.z:10.2f} |
                """
                if self.goal_pose:
                    goal_pos = self.goal_pose.position
                    dist_err = np.linalg.norm(np.array([goal_pos.x-pos.x, goal_pos.y-pos.y]))
                    heading_err_deg = math.degrees(heading_error_rad) if 'heading_error_rad' in locals() else 0.0

                    status_report += f"""
                | Goal    | {goal_pos.x:10.2f} | {goal_pos.y:10.2f} | {goal_pos.z:10.2f} |
                ----------------------------------------------------
                - State:
                  - Current Heading: {math.degrees(self.current_heading_rad):.1f} (deg)
                  - Distance Error:  {dist_err:.2f} (m)
                  - Heading Error:   {heading_err_deg:.1f} (deg)
                - PID Outputs:
                  - Altitude: {altitude_output:.1f} | Turn: {turn_output:.1f} | Forward: {forward_output:.1f}
                - Final Motor Commands (%):
                  - Left (M1): {left_motor_final:.1f} | Right (M2): {right_motor_final:.1f}
                """
                else:
                    status_report += """
                ----------------------------------------------------
                - State: NO GOAL SET
                """
                status_report += "===================================================="
                status_msg.data = status_report
                self.status_publisher.publish(status_msg)
                # Also print to console for immediate visibility
                self.get_logger().info(status_report)

def main(args=None):
    rclpy.init(args=args)
    node = BlimpNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.controller:
            node.controller.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()