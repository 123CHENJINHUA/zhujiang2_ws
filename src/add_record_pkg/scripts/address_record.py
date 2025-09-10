#!/usr/bin/env python3

import os
import rospy
import tf
import rospkg
import yaml
from geometry_msgs.msg import TransformStamped

import sys
sys.path.insert(0, "/home/cjh/miniconda3/envs/zhujiang/lib/python3.10/site-packages")

class AddressRecorder:
    def __init__(self):
        rospy.init_node('address_recorder')
        # Get the package path
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('navigation_pkg')
        self.addresses_file = os.path.join(pkg_path, 'config', 'addresses.yaml')
        self.tf_listener = tf.TransformListener()
        self.addresses = {}
        self.load_addresses()

    def load_addresses(self):
        """Load existing addresses from file if it exists"""
        try:
            # Create config directory if it doesn't exist
            os.makedirs(os.path.dirname(self.addresses_file), exist_ok=True)
            
            if os.path.exists(self.addresses_file):
                with open(self.addresses_file, 'r') as f:
                    self.addresses = yaml.safe_load(f) or {}
                rospy.loginfo("Loaded existing addresses from file")
            else:
                self.addresses = {}
                rospy.loginfo("No existing address file found, starting fresh")
        except Exception as e:
            rospy.logerr(f"Error loading addresses: {e}")
            self.addresses = {}

    def save_addresses(self):
        """Save addresses to file"""
        try:
            # Create config directory if it doesn't exist
            os.makedirs(os.path.dirname(self.addresses_file), exist_ok=True)
            
            with open(self.addresses_file, 'w') as f:
                yaml.safe_dump(self.addresses, f, default_flow_style=False)
            rospy.loginfo("Addresses saved to file")
        except Exception as e:
            rospy.logerr(f"Error saving addresses: {e}")

    def get_current_position(self):
        """Get current position from tf"""
        try:
            # 首先检查是否有可用的转换
            if not self.tf_listener.canTransform("map_3d", "body_foot_2d", rospy.Time(0)):
                rospy.logwarn("No transform available between map_3d and body_foot_2d")
                return None

            # 使用最新的可用转换
            now = rospy.Time.now()
            self.tf_listener.waitForTransform("map_3d", "body_foot_2d", now, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform("map_3d", "body_foot_2d", now)

            # 记录位置信息
            position = {
                'x': float(trans[0]),
                'y': float(trans[1]),
                'z': float(trans[2]),
                'rotation': {
                    'x': float(rot[0]),
                    'y': float(rot[1]),
                    'z': float(rot[2]),
                    'w': float(rot[3])
                },
                'timestamp': str(now.to_sec())
            }
            return position

        except (tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn(f"TF Connection Error: {e}")
            return None
        except tf.ExtrapolationException as e:
            rospy.logwarn(f"TF Timing Error: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"Unexpected error getting transform: {e}")
            return None

    def record_address(self):
        """Record a new address and its coordinates"""
        address_name = input("Enter the address name (or 'q' to quit): ")
        if address_name.lower() == 'q':
            return False

        position = self.get_current_position()
        if position:
            self.addresses[address_name] = position
            rospy.loginfo(f"Recorded address '{address_name}' at position: {position}")
            self.save_addresses()
        else:
            rospy.logerr("Failed to get current position")
        
        return True

    def list_addresses(self):
        """List all recorded addresses"""
        if not self.addresses:
            print("No addresses recorded yet")
            return

        print("\nRecorded Addresses:")
        print("-" * 50)
        for name, pos in self.addresses.items():
            print(f"\nAddress: {name}")
            print(f"Position: x={pos['x']:.3f}, y={pos['y']:.3f}, z={pos['z']:.3f}")
            print(f"Rotation: x={pos['rotation']['x']:.3f}, y={pos['rotation']['y']:.3f}, " + 
                  f"z={pos['rotation']['z']:.3f}, w={pos['rotation']['w']:.3f}")
        print("-" * 50)

    def delete_address(self, address_name):
        """Delete an address from records"""
        if address_name in self.addresses:
            del self.addresses[address_name]
            self.save_addresses()
            rospy.loginfo(f"Deleted address '{address_name}'")
            return True
        else:
            rospy.logwarn(f"Address '{address_name}' not found")
            return False

    def modify_address(self, address_name):
        """Modify an existing address with current position"""
        if address_name not in self.addresses:
            rospy.logwarn(f"Address '{address_name}' not found")
            return False

        position = self.get_current_position()
        if position:
            self.addresses[address_name] = position
            self.save_addresses()
            rospy.loginfo(f"Updated address '{address_name}' to new position: {position}")
            return True
        return False

    def run(self):
        """Main run loop"""
        print("\nAddress Recorder")
        print("Commands:")
        print("- 'add <name>' to record new position")
        print("- 'modify <name>' to update existing address")
        print("- 'delete <name>' to remove address")
        print("- 'list' to show all addresses")
        print("- 'q' to quit")

        while not rospy.is_shutdown():
            command = input("\nEnter command: ").strip()
            
            if command.lower() == 'q':
                break
            elif command.lower() == 'list':
                self.list_addresses()
            elif command.lower().startswith('add '):
                address_name = command[4:].strip()
                if address_name:
                    print(f"Getting current position for '{address_name}'...")
                    position = self.get_current_position()
                    if position:
                        self.addresses[address_name] = position
                        print(f"\nSuccessfully recorded address '{address_name}' at:")
                        print(f"  Position: x={position['x']:.3f}, y={position['y']:.3f}, z={position['z']:.3f}")
                        print(f"  Time: {position['timestamp']}")
                        self.save_addresses()
                    else:
                        print("\nFailed to get current position. Please make sure:")
                        print("1. The robot's position is being published")
                        print("2. The TF tree is properly connected")
                        print("3. The map_3d frame exists")
                else:
                    print("Please provide an address name after 'add'")
            elif command.lower().startswith('delete '):
                address_name = command[7:].strip()
                if address_name:
                    if self.delete_address(address_name):
                        print(f"\nSuccessfully deleted address '{address_name}'")
                    else:
                        print(f"\nAddress '{address_name}' not found")
                else:
                    print("Please provide an address name after 'delete'")
            elif command.lower().startswith('modify '):
                address_name = command[7:].strip()
                if address_name:
                    print(f"Getting new position for '{address_name}'...")
                    if self.modify_address(address_name):
                        position = self.addresses[address_name]
                        print(f"\nSuccessfully updated address '{address_name}' to:")
                        print(f"  Position: x={position['x']:.3f}, y={position['y']:.3f}, z={position['z']:.3f}")
                        print(f"  Time: {position['timestamp']}")
                    else:
                        if address_name in self.addresses:
                            print("\nFailed to get new position. Please make sure:")
                            print("1. The robot's position is being published")
                            print("2. The TF tree is properly connected")
                            print("3. The map_3d frame exists")
                        else:
                            print(f"\nAddress '{address_name}' not found")
                else:
                    print("Please provide an address name after 'modify'")
            else:
                print("\nUnknown command. Available commands:")
                print("- 'add <name>' to record new position")
                print("- 'modify <name>' to update existing address")
                print("- 'delete <name>' to remove address")
                print("- 'list' to show all addresses")
                print("- 'q' to quit")

if __name__ == '__main__':
    try:
        recorder = AddressRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        pass
