#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.srv import GetMap
from tf.transformations import quaternion_from_euler
import os
import sys
import signal

class NavigationAndSLAM:
    def __init__(self):
        rospy.init_node('navigation_and_slam', anonymous=True)

        # Initialize subscribers and publishers for AMCL and SLAM
        self.amcl_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Wait for the move_base action server to come up
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        rospy.loginfo("NavigationAndSlam initialized.")

    def signal_handler(self, sig, frame):
        rospy.loginfo("Shutting down...")
        sys.exit(0)

    def set_initial_pose(self, x, y, theta):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"
        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y
        quaternion = quaternion_from_euler(0, 0, theta)
        initial_pose.pose.pose.orientation.x = quaternion[0]
        initial_pose.pose.pose.orientation.y = quaternion[1]
        initial_pose.pose.pose.orientation.z = quaternion[2]
        initial_pose.pose.pose.orientation.w = quaternion[3]
        initial_pose.pose.covariance = [0.25, 0, 0, 0, 0, 0,
                                        0, 0.25, 0, 0, 0, 0,
                                        0, 0, 0.25, 0, 0, 0,
                                        0, 0, 0, 0.06853892326654787, 0, 0,
                                        0, 0, 0, 0, 0.06853892326654787, 0,
                                        0, 0, 0, 0, 0, 0.06853892326654787]

        self.amcl_pose_pub.publish(initial_pose)
        rospy.loginfo("Initial pose set.")

    def navigate_to_goal(self, x, y, theta):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        quaternion = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.move_base_client.send_goal(goal)
        wait = self.move_base_client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.move_base_client.get_result()

    def save_map(self, map_name):
        rospy.wait_for_service('/dynamic_map')
        try:
            get_map = rospy.ServiceProxy('/dynamic_map', GetMap)
            resp = get_map()
            map_file_path = os.path.join(os.path.expanduser('~'), f"{map_name}.pgm")
            with open(map_file_path, 'w') as f:
                for row in resp.map.data:
                    f.write(''.join(map(str, row)) + '\n')
            rospy.loginfo(f"Map saved as {map_file_path}")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def run_gmapping_slam(self):
        rospy.loginfo("Starting GMapping SLAM...")
        os.system("roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping")

    def run_amcl_localization(self):
        rospy.loginfo("Starting AMCL localization...")
        os.system("roslaunch turtlebot3_navigation turtlebot3_navigation.launch")

    def start_rviz(self):
        rospy.loginfo("Starting RViz for visualization...")
        os.system("roslaunch turtlebot3_navigation turtlebot3_navigation_rviz.launch")

    def go_to_multiple_goals(self):
        # Example goals
        goals = [
            (1.0, 1.0, 0.0),
            (-1.0, -1.0, 1.57),
            (2.0, 2.0, 0.0),
            (-2.0, -2.0, 1.57)
        ]
        for x, y, theta in goals:
            result = self.navigate_to_goal(x, y, theta)
            if result:
                rospy.loginfo("Goal reached successfully.")
            else:
                rospy.logwarn("Failed to reach goal.")

if __name__ == '__main__':
    try:
        nav_slam = NavigationAndSLAM()
        signal.signal(signal.SIGINT, nav_slam.signal_handler)

        # Start RViz
        nav_slam.start_rviz()

        # Start GMapping SLAM
        nav_slam.run_gmapping_slam()
        rospy.sleep(10)  # Give some time for the SLAM to start

        # Set initial pose
        nav_slam.set_initial_pose(0.0, 0.0, 0.0)

        # Navigate to multiple goals
        nav_slam.go_to_multiple_goals()

        # Save the map
        nav_slam.save_map("my_map")

        # Stop SLAM and switch to AMCL
        os.system("rosnode kill -a")  # Stopping all nodes
        rospy.sleep(5)
        
        nav_slam.run_amcl_localization()
        rospy.sleep(10)  # Give some time for the localization to start

        # Set initial pose again for localization
        nav_slam.set_initial_pose(0.0, 0.0, 0.0)

        # Navigate to some goals using AMCL
        nav_slam.go_to_multiple_goals()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation and SLAM test finished.")
