#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

print("\n" + "="*70)
print("ROBODK + ROS2 INTEGRATION TEST")
print("="*70)

# Test imports
print("\n[1/4] Testing imports...")
try:
    from robodk import robolink, robomath
    print("  ✓ robodk imported")
except ImportError as e:
    print(f"  ✗ Failed: {e}")
    exit(1)

# Test RoboDK connection
print("\n[2/4] Connecting to RoboDK...")
try:
    RDK = robolink.Robolink()
    print("  ✓ Connected")
except Exception as e:
    print(f"  ✗ Failed: {e}")
    exit(1)

# Test robot access
print("\n[3/4] Finding robots...")
try:
    master = RDK.Item("Master", robolink.ITEM_TYPE_ROBOT)
    slave = RDK.Item("Slave", robolink.ITEM_TYPE_ROBOT)
    
    if master.Valid():
        print(f"  ✓ Master found")
    else:
        print(f"  ✗ Master not found")
        exit(1)
    
    if slave.Valid():
        print(f"  ✓ Slave found")
    else:
        print(f"  ✗ Slave not found")
        exit(1)
        
except Exception as e:
    print(f"  ✗ Error: {e}")
    exit(1)

# Test reading joints
print("\n[4/4] Testing joint reading...")
try:
    m_joints_raw = master.Joints().tr()[0]
    s_joints_raw = slave.Joints().tr()[0]
    
    # Flatten nested arrays
    def flatten_joints(joints):
        result = []
        for j in joints:
            if isinstance(j, (list, tuple)):
                result.extend(flatten_joints(j))
            else:
                result.append(float(j))
        return result
    
    m_joints = flatten_joints(m_joints_raw)
    s_joints = flatten_joints(s_joints_raw)
    
    print(f"  ✓ Master: {len(m_joints)} axes - A1={m_joints[0]:.1f}°")
    print(f"  ✓ Slave:  {len(s_joints)} axes - A1={s_joints[0]:.1f}°")
    
except Exception as e:
    print(f"  ✗ Error: {e}")
    exit(1)

# ROS2 Test Node
print("\n" + "="*70)
print("Starting ROS2 publishing test...")
print("="*70)

class TestNode(Node):
    def __init__(self, master_robot, slave_robot):
        super().__init__('test_node')
        
        self.master = master_robot
        self.slave = slave_robot
        
        self.master_pub = self.create_publisher(
            JointState, '/master/joint_states', 10)
        self.slave_pub = self.create_publisher(
            JointState, '/slave/joint_states', 10)
        
        self.timer = self.create_timer(0.5, self.publish_callback)
        self.count = 0
        
        self.get_logger().info("✓ ROS2 node ready")
        self.get_logger().info("✓ Publishing to /master/joint_states")
        self.get_logger().info("✓ Publishing to /slave/joint_states")
        self.get_logger().info("Press Ctrl+C to stop...")
    
    def flatten_joints(self, joints):
        """Flatten nested joint arrays"""
        result = []
        for j in joints:
            if isinstance(j, (list, tuple)):
                result.extend(self.flatten_joints(j))
            else:
                result.append(float(j))
        return result
    
    def publish_callback(self):
        try:
            # Read from RoboDK
            m_joints_raw = self.master.Joints().tr()[0]
            s_joints_raw = self.slave.Joints().tr()[0]
            
            # Flatten nested arrays
            m_joints = self.flatten_joints(m_joints_raw)
            s_joints = self.flatten_joints(s_joints_raw)
            
            # Ensure we have 9 joints
            while len(m_joints) < 9:
                m_joints.append(0.0)
            while len(s_joints) < 9:
                s_joints.append(0.0)
            
            # Publish Master
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "master_base"
            msg.name = ['master_a1', 'master_a2', 'master_a3', 
                       'master_a4', 'master_a5', 'master_a6',
                       'master_e1', 'master_e2', 'master_e3']
            
            # Convert: degrees to radians, mm to meters
            msg.position = [
                m_joints[0] * 0.017453293,  # A1
                m_joints[1] * 0.017453293,  # A2
                m_joints[2] * 0.017453293,  # A3
                m_joints[3] * 0.017453293,  # A4
                m_joints[4] * 0.017453293,  # A5
                m_joints[5] * 0.017453293,  # A6
                m_joints[6] * 0.001,         # E1 (mm to m)
                m_joints[7] * 0.017453293,  # E2
                m_joints[8] * 0.017453293   # E3
            ]
            
            self.master_pub.publish(msg)
            
            # Publish Slave
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
            
            self.slave_pub.publish(msg)
            
            self.count += 1
            
            if self.count % 4 == 0:  # Every 2 seconds
                self.get_logger().info(
                    f'[{self.count}] Master: A1={m_joints[0]:.1f}° E1={m_joints[6]:.0f}mm | '
                    f'Slave: A1={s_joints[0]:.1f}° E1={s_joints[6]:.0f}mm'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main():
    rclpy.init()
    
    node = TestNode(master, slave)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print(f"✓✓✓ TEST COMPLETED ✓✓✓")
        print(f"Published {node.count} messages successfully")
        print("RoboDK → ROS2 integration working!")
        print("="*70)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
