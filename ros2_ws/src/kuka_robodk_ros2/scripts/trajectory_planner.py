#!/usr/bin/env python3

"""
Trajectory Planner for Dual KUKA Robots
Sends planned trajectories to RoboDK via ROS2
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')
        
        self.master_pub = self.create_publisher(
            JointTrajectory, '/master/joint_trajectory', 10)
        
        self.slave_pub = self.create_publisher(
            JointTrajectory, '/slave/joint_trajectory', 10)
        
        time.sleep(1)  # Wait for publishers
        
        self.get_logger().info("="*70)
        self.get_logger().info("TRAJECTORY PLANNER READY")
        self.get_logger().info("="*70)
    
    def create_trajectory(self, waypoints, joint_names, durations):
        """
        Create a JointTrajectory message
        
        waypoints: List of joint positions (in radians/meters)
        joint_names: List of joint names
        durations: List of time from start for each waypoint (seconds)
        """
        traj = JointTrajectory()
        traj.joint_names = joint_names
        
        for i, (pos, dur) in enumerate(zip(waypoints, durations)):
            point = JointTrajectoryPoint()
            point.positions = pos
            point.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
            traj.points.append(point)
        
        return traj
    
    def demo_home_sequence(self):
        """Demo: Move both robots to home position"""
        self.get_logger().info("\n[DEMO 1] Home Sequence")
        self.get_logger().info("-" * 70)
        
        joint_names = ['master_a1', 'master_a2', 'master_a3',
                      'master_a4', 'master_a5', 'master_a6',
                      'master_e1', 'master_e2', 'master_e3']
        
        # Home position: A1-A6 = [0, -90, 90, 0, 0, 0], E1-E3 = [0, 0, 0]
        home = [0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        waypoints = [home]
        durations = [3.0]  # 3 seconds to reach home
        
        traj = self.create_trajectory(waypoints, joint_names, durations)
        
        self.get_logger().info("Sending Master to home...")
        self.master_pub.publish(traj)
        
        # Same for slave
        traj.joint_names = ['slave_a1', 'slave_a2', 'slave_a3',
                           'slave_a4', 'slave_a5', 'slave_a6',
                           'slave_e1', 'slave_e2', 'slave_e3']
        
        self.get_logger().info("Sending Slave to home...")
        self.slave_pub.publish(traj)
        
        time.sleep(4)
        self.get_logger().info("✓ Home sequence complete")
    
    def demo_turntable_rotation(self):
        """Demo: Rotate turntables while robots follow"""
        self.get_logger().info("\n[DEMO 2] Turntable Rotation")
        self.get_logger().info("-" * 70)
        
        joint_names = ['master_a1', 'master_a2', 'master_a3',
                      'master_a4', 'master_a5', 'master_a6',
                      'master_e1', 'master_e2', 'master_e3']
        
        # Waypoints: Rotate turntable E2 through 0° → 45° → 90° → 0°
        waypoints = [
            [0.0, -1.0472, 1.3963, 0.0, 0.5236, 0.0, 0.0, 0.0, 0.0],           # 0°
            [0.0, -1.0472, 1.3963, 0.0, 0.5236, 0.0, 0.0, 0.7854, 0.0],        # 45°
            [0.0, -1.0472, 1.3963, 0.0, 0.5236, 0.0, 0.0, 1.5708, 0.0],        # 90°
            [0.0, -1.0472, 1.3963, 0.0, 0.5236, 0.0, 0.0, 0.0, 0.0],           # back to 0°
        ]
        
        durations = [2.0, 4.0, 6.0, 8.0]
        
        traj = self.create_trajectory(waypoints, joint_names, durations)
        
        self.get_logger().info("Rotating Master turntable...")
        self.master_pub.publish(traj)
        
        # Same for slave
        traj.joint_names = ['slave_a1', 'slave_a2', 'slave_a3',
                           'slave_a4', 'slave_a5', 'slave_a6',
                           'slave_e1', 'slave_e2', 'slave_e3']
        
        self.get_logger().info("Rotating Slave turntable...")
        self.slave_pub.publish(traj)
        
        time.sleep(9)
        self.get_logger().info("✓ Turntable rotation complete")
    
    def demo_track_movement(self):
        """Demo: Move tracks in synchronization"""
        self.get_logger().info("\n[DEMO 3] Track Movement")
        self.get_logger().info("-" * 70)
        
        joint_names = ['master_a1', 'master_a2', 'master_a3',
                      'master_a4', 'master_a5', 'master_a6',
                      'master_e1', 'master_e2', 'master_e3']
        
        # Move tracks: 0mm → 1000mm → 2000mm → 0mm
        waypoints = [
            [0.0, -1.2217, 1.4835, 0.0, 0.6109, 0.0, 0.0, 0.0, 0.0],       # Track at 0mm
            [0.0, -1.2217, 1.4835, 0.0, 0.6109, 0.0, 1.0, 0.0, 0.0],       # Track at 1000mm
            [0.0, -1.2217, 1.4835, 0.0, 0.6109, 0.0, 2.0, 0.0, 0.0],       # Track at 2000mm
            [0.0, -1.2217, 1.4835, 0.0, 0.6109, 0.0, 0.0, 0.0, 0.0],       # Back to 0mm
        ]
        
        durations = [3.0, 6.0, 9.0, 12.0]
        
        traj = self.create_trajectory(waypoints, joint_names, durations)
        
        self.get_logger().info("Moving Master track...")
        self.master_pub.publish(traj)
        
        # Slave moves opposite direction
        slave_waypoints = [
            [0.0, -1.2217, 1.4835, 0.0, 0.6109, 0.0, 0.0, 0.0, 0.0],
            [0.0, -1.2217, 1.4835, 0.0, 0.6109, 0.0, -1.0, 0.0, 0.0],
            [0.0, -1.2217, 1.4835, 0.0, 0.6109, 0.0, -2.0, 0.0, 0.0],
            [0.0, -1.2217, 1.4835, 0.0, 0.6109, 0.0, 0.0, 0.0, 0.0],
        ]
        
        traj.joint_names = ['slave_a1', 'slave_a2', 'slave_a3',
                           'slave_a4', 'slave_a5', 'slave_a6',
                           'slave_e1', 'slave_e2', 'slave_e3']
        traj.points = []
        for pos, dur in zip(slave_waypoints, durations):
            point = JointTrajectoryPoint()
            point.positions = pos
            point.time_from_start = Duration(sec=int(dur), nanosec=int((dur % 1) * 1e9))
            traj.points.append(point)
        
        self.get_logger().info("Moving Slave track...")
        self.slave_pub.publish(traj)
        
        time.sleep(13)
        self.get_logger().info("✓ Track movement complete")
    
    def run_demo(self):
        """Run complete demo sequence"""
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("STARTING TRAJECTORY DEMO")
        self.get_logger().info("="*70)
        
        self.demo_home_sequence()
        time.sleep(2)
        
        self.demo_turntable_rotation()
        time.sleep(2)
        
        self.demo_track_movement()
        time.sleep(2)
        
        self.demo_home_sequence()
        
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("✓✓✓ DEMO COMPLETE ✓✓✓")
        self.get_logger().info("="*70)

def main():
    rclpy.init()
    
    planner = TrajectoryPlanner()
    planner.run_demo()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
