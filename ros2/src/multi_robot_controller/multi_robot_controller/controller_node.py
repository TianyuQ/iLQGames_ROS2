import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

# from juliacall import Main as jl
# import julia
from julia.api import Julia
jl = Julia(compiled_modules=False)
from julia import Main

# Only do this once — import and include your Julia code
def julia_init():
    print("Initializing Julia...")

    jl.eval('using Pkg')
    jl.eval('Pkg.activate("/home/tq877/Tianyu/Robot_Experiments/iLQGames_ROS2/test")')
    jl.eval('Pkg.instantiate()')
    # jl.eval('Main.include("/home/tq877/Tianyu/Robot Experiments/MCP_Isaac_sim_ros2/examples/TrajectoryExamples.jl")')
    Main.include("/home/tq877/Tianyu/Robot_Experiments/iLQGames_ROS2/test/test_nplayer_navigation.jl")
    print("Julia initialized.")
    print("Solver Warming up...")
    # _ = Main.nplayer_navigation()
    print("Solver Warming up done.")

def goal_reached(posittion, goal_position, threshold=0.2):
    """
    Check if the robot has reached the goal position within a threshold.
    """
    distance = math.sqrt((posittion[0] - goal_position[0]) ** 2 + (posittion[1] - goal_position[1]) ** 2)
    return distance < threshold


class MultiRobotController(Node):
    def __init__(self):
        super().__init__('multi_robot_controller')

        # Publishers
        self.cmd_pub_01 = self.create_publisher(Twist, '/cmd_vel_01', 10)
        self.cmd_pub_02 = self.create_publisher(Twist, '/cmd_vel_02', 10)
        self.cmd_pub_03 = self.create_publisher(Twist, '/cmd_vel_03', 10)  # Placeholder for third robot if needed

        # Subscribers
        self.odom_sub_01 = self.create_subscription(
            Odometry, '/odom_01', self.odom_callback_01, 10)
        self.odom_sub_02 = self.create_subscription(
            Odometry, '/odom_02', self.odom_callback_02, 10)
        self.odom_sub_03 = self.create_subscription(
            Odometry, '/odom_03', self.odom_callback_03, 10)  # Placeholder for third robot if needed

        # Odometry buffers
        self.latest_odom_01 = None
        self.latest_odom_02 = None
        self.latest_odom_03 = None  # Placeholder for third robot if needed

        # Timer to run planner at 10 Hz
        self.timer = self.create_timer(0.1, self.run_planner_step)

        self.get_logger().info("MultiRobotController node started.")

    def odom_callback_01(self, msg):
        self.latest_odom_01 = msg

    def odom_callback_02(self, msg):
        self.latest_odom_02 = msg

    def odom_callback_03(self, msg):
        self.latest_odom_03 = msg

    def convert_odom_to_state(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw (theta)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        v = msg.twist.twist.linear.x

        return [x, y, theta, v]

    def convert_to_cmd_vel(self, vx, vy, pose, target_pose, current_theta):
        v = math.hypot(vx, vy)
        # target_theta = math.atan2(vy, vx)
        target_theta = math.atan2(target_pose[1] - pose[1], target_pose[0] - pose[0])
        print("target_theta:", target_theta)
        print("current_theta:", current_theta)
        # if vy >= 0:
        #     target_theta = math.asin(vy, v)
        # else:
        #     target_theta = math.pi - math.asin(vy, vx)

        omega = (target_theta - current_theta)  # Assuming a time step of 0.1 seconds
        
        return v, omega

    def run_planner_step(self):
        if self.latest_odom_01 is None or self.latest_odom_02 is None:
            self.get_logger().warn("Waiting for odometry...")
            return

        try:
            state1 = self.convert_odom_to_state(self.latest_odom_01)
            state2 = self.convert_odom_to_state(self.latest_odom_02)
            state3 = self.convert_odom_to_state(self.latest_odom_03)  # Placeholder for third robot if needed

            # print("state1:", state1)
            # print("state2:", state2)

            initial_state = state1 + state2 + state3  # Concatenate states for all robots

            # julia_state = Main.mortar(initial_state)
            result = Main.nplayer_navigation(initial_state)

            v1, omega1 = result[0][3], result[1][0]
            v2, omega2 = result[0][7], result[1][2]
            v3, omega3 = result[0][11], result[1][4]  # Placeholder for third robot if needed

            np.clip(omega1, -0.5, 0.5)
            np.clip(omega2, -0.5, 0.5)
            np.clip(omega3, -0.5, 0.5)  # Placeholder for third robot if needed

            twist1 = Twist()
            twist1.linear.x = v1
            twist1.angular.z = omega1  # optional: compute omega

            twist2 = Twist()
            twist2.linear.x = v2
            twist2.angular.z = omega2

            twist3 = Twist()  # Placeholder for third robot if needed
            twist3.linear.x = v3
            twist3.angular.z = omega3

            if not goal_reached(state1[:2], [3, 0]):
                self.cmd_pub_01.publish(twist1)
            else:
                self._logger.info("Goal reached for robot 1, stopping.")
                twist1.linear.x = 0.0
                twist1.angular.z = 0.0
                self.cmd_pub_01.publish(twist1)

            if not goal_reached(state2[:2], [0, -3]):
                self.cmd_pub_02.publish(twist2)
            else:
                self._logger.info("Goal reached for robot 2, stopping.")
                twist2.linear.x = 0.0
                twist2.angular.z = 0.0
                self.cmd_pub_02.publish(twist2)

            if not goal_reached(state3[:2], [3, -3]):
                self.cmd_pub_03.publish(twist3)
            else:
                self._logger.info("Goal reached for robot 3, stopping.")
                twist3.linear.x = 0.0
                twist3.angular.z = 0.0
                self.cmd_pub_03.publish(twist3)

            # self.get_logger().info(f"Published: v1={v1:.2f}, v2={v2:.2f}")
            # self.get_logger().info(f"Published: omega1={omega1:.2f}, omega2={omega2:.2f}")

        except Exception as e:
            self.get_logger().error(f"Julia error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    julia_init()
    main()
