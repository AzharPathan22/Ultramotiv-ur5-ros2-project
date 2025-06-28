import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ikpy.chain import Chain
import numpy as np
import os


class UR5IKCommandNode(Node):
    def __init__(self):
        super().__init__('ur5_ik_command_node')

        # Declare parameters for target XYZ position
        self.declare_parameter('x', 0.4)
        self.declare_parameter('y', 0.2)
        self.declare_parameter('z', 0.5)

        self.get_logger().info("Loading URDF for IK...")

        # Load robot kinematic chain from URDF
        urdf_path = "/home/azhar/ultramotiv_ws/src/ur5_sim/urdf/ur5_custom.urdf"
        self.chain = Chain.from_urdf_file(
            urdf_path,
            active_links_mask=[False, True, True, True, True, True, True, False]
        )

        # Define joint names for the trajectory
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]

        # ROS 2 publisher for joint trajectories
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Timer to periodically send IK command (every 2 seconds)
        self.timer = self.create_timer(2.0, self.send_ik_command)

    def send_ik_command(self):
        # Retrieve target pose from declared parameters
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        z = self.get_parameter('z').value
        target = [x, y, z]

        # Compute inverse kinematics to get joint angles
        joint_angles = self.chain.inverse_kinematics(target)

        # Logging target and IK result
        self.get_logger().info(f"Target: {target}")
        self.get_logger().info(f"Computed IK Angles: {joint_angles[1:7]}")

        # Create JointTrajectory message
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_angles[1:7].tolist()  # Skip the base joint (index 0)
        point.time_from_start.sec = 2  # Reach target in 2 seconds

        msg.points.append(point)

        # Publish the joint trajectory
        self.publisher.publish(msg)
        self.get_logger().info("Trajectory command sent!")

        # Optional: Cancel the timer if only one execution is needed
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = UR5IKCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
