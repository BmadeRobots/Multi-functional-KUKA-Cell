#!/usr/bin/env python3

"""
Dual KUKA Bidirectional ROS2-RoboDK Node
- Publishes joint states from RoboDK (50Hz)
- Receives trajectory commands from ROS2
- Executes trajectories in RoboDK
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from robodk import robolink, robomath
import threading
import time

# ============================================================================
# CONFIGURATION
# ============================================================================
PUBLISH_RATE = 50  # Hz
ROBOT_SPEED = 50   # mm/s
ROBOT_ACCEL = 20   # mm/s²

# ============================================================================
# ROBODK INTERFACE
# ============================================================================
class RoboDKInterface:
    """Interface to RoboDK robots"""
    
    def __init__(self, logger):
        self.logger = logger
        self.RDK = robolink.Robolink()
        
        # Get robots
        self.master = self.RDK.Item("Master", robolink.ITEM_TYPE_ROBOT)
        self.slave = self.RDK.Item("Slave", robolink.ITEM_TYPE_ROBOT)
        
        if not self.master.Valid() or not self.slave.Valid():
            raise Exception("Master or Slave robot not found in RoboDK!")
        
        # Configure speeds
        self.master.setSpeed(ROBOT_SPEED)
        self.master.setAcceleration(ROBOT_ACCEL)
        self.slave.setSpeed(ROBOT_SPEED)
        self.slave.setAcceleration(ROBOT_ACCEL)
        
        self.logger.info(f"✓ RoboDK robots initialized")
        self.logger.info(f"  Master: {self.master.Name()}")
        self.logger.info(f"  Slave: {self.slave.Name()}")
        self.logger.info(f"  Speed: {ROBOT_SPEED} mm/s")
    
    def flatten_joints(self, joints):
        """Flatten nested joint arrays from RoboDK"""
        result = []
        for j in joints:
            if isinstance(j, (list, tuple)):
                result.extend(self.flatten_joints(j))
            else:
                result.append(float(j))
        return result
    
    def get_master_joints(self):
        """Get Master robot joint positions"""
        try:
            joints_raw = self.master.Joints().tr()[0]
            joints = self.flatten_joints(joints_raw)
            
            # Ensure 9 joints
            while len(joints) < 9:
                joints.append(0.0)
            
            return joints[:9]
        except Exception as e:
            self.logger.error(f"Error reading Master joints: {e}")
            return None
    
    def get_slave_joints(self):
        """Get Slave robot joint positions"""
        try:
            joints_raw = self.slave.Joints().tr()[0]
            joints = self.flatten_joints(joints_raw)
            
            while len(joints) < 9:
                joints.append(0.0)
            
            return joints[:9]
        except Exception as e:
            self.logger.error(f"Error reading Slave joints: {e}")
            return None
    
    def move_master_joints(self, joints, blocking=False):
        """Move Master robot to joint positions"""
        try:
            joints_mat = robomath.Mat([joints]).tr()
            self.master.MoveJ(joints_mat, blocking=blocking)
            return True
        except Exception as e:
            self.logger.error(f"Error moving Master: {e}")
            return False
    
    def move_slave_joints(self, joints, blocking=False):
        """Move Slave robot to joint positions"""
        try:
            joints_mat = robomath.Mat([joints]).tr()
            self.slave.MoveJ(joints_mat, blocking=blocking)
            return True
        except Exception as e:
            self.logger.error(f"Error moving Slave: {e}")
            return False

# ============================================================================
# BIDIRECTIONAL ROS2 NODE
# ============================================================================
class DualKUKABidirectionalNode(Node):
    """ROS2 Node with bidirectional RoboDK control"""
    
    def __init__(self):
        super().__init__('dual_kuka_bidirectional_node')
        
        self.get_logger().info("="*70)
        self.get_logger().info("DUAL KUKA BIDIRECTIONAL NODE")
        self.get_logger().info("="*70)
        
        # Initialize RoboDK
        try:
            self.robodk = RoboDKInterface(self.get_logger())
        except Exception as e:
            self.get_logger().error(f"Failed to initialize RoboDK: {e}")
            raise
        
        # ====================================================================
        # PUBLISHERS - Joint States (RoboDK → ROS2)
        # ====================================================================
        self.master_state_pub = self.create_publisher(
            JointState, '/master/joint_states', 10)
        
        self.slave_state_pub = self.create_publisher(
            JointState, '/slave/joint_states', 10)
        
        # ====================================================================
        # SUBSCRIBERS - Trajectory Commands (ROS2 → RoboDK)
        # ====================================================================
        self.master_traj_sub = self.create_subscription(
            JointTrajectory,
            '/master/joint_trajectory',
            self.master_trajectory_callback,
            10
        )
        
        self.slave_traj_sub = self.create_subscription(
            JointTrajectory,
            '/slave/joint_trajectory',
            self.slave_trajectory_callback,
            10
        )
        
        # Simple position commands
        self.master_cmd_sub = self.create_subscription(
            JointState,
            '/master/joint_command',
            self.master_command_callback,
            10
        )
        
        self.slave_cmd_sub = self.create_subscription(
            JointState,
            '/slave/joint_command',
            self.slave_command_callback,
            10
        )
        
        # ====================================================================
        # TIMER - Publish joint states
        # ====================================================================
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.publish_states)
        
        self.get_logger().info("✓ Publishers:")
        self.get_logger().info("  - /master/joint_states")
        self.get_logger().info("  - /slave/joint_states")
        self.get_logger().info("✓ Subscribers:")
        self.get_logger().info("  - /master/joint_trajectory")
        self.get_logger().info("  - /slave/joint_trajectory")
        self.get_logger().info("  - /master/joint_command")
        self.get_logger().info("  - /slave/joint_command")
        self.get_logger().info(f"✓ Publishing rate: {PUBLISH_RATE} Hz")
        self.get_logger().info("="*70)
        self.get_logger().info("Ready to receive commands!")
        self.get_logger().info("="*70)
    
    # ========================================================================
    # PUBLISHING (RoboDK → ROS2)
    # ========================================================================
    def publish_states(self):
        """Publish current joint states from RoboDK"""
        
        # Get joints from RoboDK
        m_joints = self.robodk.get_master_joints()
        s_joints = self.robodk.get_slave_joints()
        
        # Publish Master
        if m_joints:
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "master_base"
            msg.name = ['master_a1', 'master_a2', 'master_a3',
                       'master_a4', 'master_a5', 'master_a6',
                       'master_e1', 'master_e2', 'master_e3']
            
            # Convert: deg→rad, mm→m
            msg.position = [
                m_joints[0] * 0.017453293,
                m_joints[1] * 0.017453293,
                m_joints[2] * 0.017453293,
                m_joints[3] * 0.017453293,
                m_joints[4] * 0.017453293,
                m_joints[5] * 0.017453293,
                m_joints[6] * 0.001,
                m_joints[7] * 0.017453293,
                m_joints[8] * 0.017453293
            ]
            
            self.master_state_pub.publish(msg)
        
        # Publish Slave
        if s_joints:
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "slave_base"
            msg.name = ['slave_a1', 'slave_a2', 'slave_a3',
                       'slave_a4', 'slave_a5', 'slave_a6',
                       'slave_e1', 'slave_e2', 'slave_e3']
            
            msg.position = [
                s_joints[0] * 0.017453293,
                s_joints[1] * 0.017453293,
                s_joints[2] * 0.017453293,
                s_joints[3] * 0.017453293,
                s_joints[4] * 0.017453293,
                s_joints[5] * 0.017453293,
                s_joints[6] * 0.001,
                s_joints[7] * 0.017453293,
                s_joints[8] * 0.017453293
            ]
            
            self.slave_state_pub.publish(msg)
    
    # ========================================================================
    # TRAJECTORY COMMANDS (ROS2 → RoboDK)
    # ========================================================================
    def master_trajectory_callback(self, msg):
        """Execute Master trajectory"""
        self.get_logger().info(f"Received Master trajectory with {len(msg.points)} points")
        
        # Execute each point in trajectory
        for i, point in enumerate(msg.points):
            if len(point.positions) < 9:
                self.get_logger().warn(f"Point {i}: Need 9 joints, got {len(point.positions)}")
                continue
            
            # Convert: rad→deg, m→mm
            joints = [
                point.positions[0] * 57.295779513,
                point.positions[1] * 57.295779513,
                point.positions[2] * 57.295779513,
                point.positions[3] * 57.295779513,
                point.positions[4] * 57.295779513,
                point.positions[5] * 57.295779513,
                point.positions[6] * 1000.0,
                point.positions[7] * 57.295779513,
                point.positions[8] * 57.295779513
            ]
            
            # Move to point
            success = self.robodk.move_master_joints(joints, blocking=True)
            
            if success:
                self.get_logger().info(f"  Point {i+1}/{len(msg.points)} ✓")
            else:
                self.get_logger().error(f"  Point {i+1}/{len(msg.points)} ✗")
                break
        
        self.get_logger().info("Master trajectory complete")
    
    def slave_trajectory_callback(self, msg):
        """Execute Slave trajectory"""
        self.get_logger().info(f"Received Slave trajectory with {len(msg.points)} points")
        
        for i, point in enumerate(msg.points):
            if len(point.positions) < 9:
                self.get_logger().warn(f"Point {i}: Need 9 joints, got {len(point.positions)}")
                continue
            
            joints = [
                point.positions[0] * 57.295779513,
                point.positions[1] * 57.295779513,
                point.positions[2] * 57.295779513,
                point.positions[3] * 57.295779513,
                point.positions[4] * 57.295779513,
                point.positions[5] * 57.295779513,
                point.positions[6] * 1000.0,
                point.positions[7] * 57.295779513,
                point.positions[8] * 57.295779513
            ]
            
            success = self.robodk.move_slave_joints(joints, blocking=True)
            
            if success:
                self.get_logger().info(f"  Point {i+1}/{len(msg.points)} ✓")
            else:
                self.get_logger().error(f"  Point {i+1}/{len(msg.points)} ✗")
                break
        
        self.get_logger().info("Slave trajectory complete")
    
    # ========================================================================
    # SIMPLE POSITION COMMANDS (ROS2 → RoboDK)
    # ========================================================================
    def master_command_callback(self, msg):
        """Move Master to position"""
        
        if len(msg.position) < 9:
            self.get_logger().warn(f"Need 9 joints, got {len(msg.position)}")
            return
        
        # Convert: rad→deg, m→mm
        joints = [
            msg.position[0] * 57.295779513,
            msg.position[1] * 57.295779513,
            msg.position[2] * 57.295779513,
            msg.position[3] * 57.295779513,
            msg.position[4] * 57.295779513,
            msg.position[5] * 57.295779513,
            msg.position[6] * 1000.0,
            msg.position[7] * 57.295779513,
            msg.position[8] * 57.295779513
        ]
        
        self.robodk.move_master_joints(joints, blocking=False)
        self.get_logger().info(f"Master commanded: A1={joints[0]:.1f}° E1={joints[6]:.0f}mm")
    
    def slave_command_callback(self, msg):
        """Move Slave to position"""
        
        if len(msg.position) < 9:
            self.get_logger().warn(f"Need 9 joints, got {len(msg.position)}")
            return
        
        joints = [
            msg.position[0] * 57.295779513,
            msg.position[1] * 57.295779513,
            msg.position[2] * 57.295779513,
            msg.position[3] * 57.295779513,
            msg.position[4] * 57.295779513,
            msg.position[5] * 57.295779513,
            msg.position[6] * 1000.0,
            msg.position[7] * 57.295779513,
            msg.position[8] * 57.295779513
        ]
        
        self.robodk.move_slave_joints(joints, blocking=False)
        self.get_logger().info(f"Slave commanded: A1={joints[0]:.1f}° E1={joints[6]:.0f}mm")

# ============================================================================
# MAIN
# ============================================================================
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DualKUKABidirectionalNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
