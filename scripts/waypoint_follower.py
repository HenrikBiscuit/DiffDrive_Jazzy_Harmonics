#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
import sys
import time
import math
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
    """
    Creates a geographic_msgs/msg/GeoPose object from latitude, longitude and yaw
    """
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose

class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """
    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps

class GpsWpCommander:
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """
    def __init__(self, wps_file_path):
        rclpy.init()
        self.node = rclpy.create_node('gps_waypoint_follower')
        self.navigator = BasicNavigator()
        self.wp_parser = YamlWaypointParser(wps_file_path)
        # Create lifecycle state client
        from lifecycle_msgs.srv import GetState
        self.get_state_client = self.node.create_client(GetState, '/bt_navigator/get_state')

    def wait_for_topics(self):
        """
        Wait for required topics to become available
        """
        required_topics = [
            '/odom',
            '/cmd_vel',
            '/odometry/global',
            '/odometry/local',
            '/navsat'
        ]
        
        print('Waiting for required topics...')
        while rclpy.ok():
            available_topics = [t[0] for t in self.node.get_topic_names_and_types()]
            missing_topics = [topic for topic in required_topics if topic not in available_topics]
            if not missing_topics:
                break
            print(f'Waiting for topics: {", ".join(missing_topics)}')
            time.sleep(1)
            
        print('All required topics are available')

    def wait_for_navigation_ready(self):
        """
        Wait for the navigation stack to be ready
        """
        self.wait_for_topics()
        
        # Wait for Nav2 lifecycle nodes to be active
        print('Waiting for Nav2...')
        
        # Check for specific Nav2 services that indicate readiness
        required_services = [
            '/bt_navigator/get_state',
            '/controller_server/get_state',
            '/planner_server/get_state',
            '/behavior_server/get_state'
        ]
        
        while rclpy.ok():
            available_services = [srv[0] for srv in self.node.get_service_names_and_types()]
            missing_services = [srv for srv in required_services if srv not in available_services]
            
            if not missing_services:
                print("All Nav2 services are available")
                break
                
            print(f'Waiting for Nav2 services: {", ".join(missing_services)}')
            time.sleep(1)
        
        # Check current states of all Nav2 nodes
        print("\nChecking Nav2 node states:")
        self.check_lifecycle_states()
        
        # Now wait for the lifecycle nodes to be active
        print("\nWaiting for Nav2 nodes to be active...")
        from lifecycle_msgs.srv import GetState
        req = GetState.Request()
        
        try:
            # Wait for bt_navigator to be active
            while rclpy.ok():
                future = self.get_state_client.call_async(req)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
                if future.result() is not None:
                    state = future.result().current_state.label
                    if state == 'active':
                        break
                    print(f"bt_navigator state is: {state}")
                print("Waiting for bt_navigator to be active...")
                time.sleep(1)
            
            print("Navigation system is ready!")
            
        except Exception as e:
            print(f"Warning during Nav2 activation check: {e}")
            print("Continuing anyway as services are available...")

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        try:
            self.wait_for_navigation_ready()
            wps = self.wp_parser.get_wps()
            
            print(f"Following {len(wps)} waypoints...")
            print("Waypoint details:")
            for i, wp in enumerate(wps):
                print(f"  Waypoint {i+1}:")
                print(f"    Latitude: {wp.position.latitude}")
                print(f"    Longitude: {wp.position.longitude}")
            
            self.navigator.followGpsWaypoints(wps)
            
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                if feedback:
                    print(f'Executing current waypoint: {feedback.current_waypoint + 1}/{len(wps)}')
                time.sleep(1)
                
            print("Waypoints completed successfully!")
        except Exception as e:
            print(f"Error during waypoint following: {str(e)}")
            raise
        finally:
            self.node.destroy_node()

def main():
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        print("Error: Please provide the path to the waypoints yaml file")
        sys.exit(1)
    
    try:
        gps_wpf = GpsWpCommander(yaml_file_path)
        gps_wpf.start_wpf()
    except KeyboardInterrupt:
        print("\nNavigation cancelled by user")
    except Exception as e:
        print(f"An error occurred: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == "__main__":
    main()