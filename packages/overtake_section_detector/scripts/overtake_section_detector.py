#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from overtake_section_detector.msg import OvertakeSectionMsg 

class OvertakeDetector:
    def __init__(self):
        rospy.init_node('overtake_section_detector', anonymous=True)

        self.CURVATURE_THRESHOLD = 0.1
        self.MIN_SECTION_LENGTH = 5
        
        self.global_path_poses = []
        self.straight_sections = [] 
        self.current_pose = None
        
        self.pose_topic = '/current_pose' 
        self.path_topic = '/global_path'

        rospy.Subscriber(self.path_topic, Path, self.path_callback)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)

        self.opp_pub = rospy.Publisher('/overtake_opportunity', OvertakeSectionMsg, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def path_callback(self, data):
        self.global_path_poses = data.poses
        if len(self.global_path_poses) < 2:
            return
        
        self.straight_sections = self.analyze_curvature(self.global_path_poses)

    def pose_callback(self, data):
        self.current_pose = data

    def timer_callback(self, event):
        if not self.global_path_poses or self.current_pose is None:
            return

        current_idx = self.find_closest_waypoint(self.current_pose, self.global_path_poses)
        
        is_possible = False
        target_section = []
        
        for (start, end) in self.straight_sections:
            if start <= current_idx <= end:
                is_possible = True
                target_section = [start, end]
                break
        
        msg = OvertakeSectionMsg()
        msg.is_possible = is_possible
        msg.wp_indices = target_section if is_possible else []
        msg.curvature_score = 0.0
        
        self.opp_pub.publish(msg)

    def analyze_curvature(self, waypoints):
        straight_segments = []
        is_in_straight = False
        start_index = -1
        
        for i in range(1, len(waypoints) - 1):
            wp_prev = waypoints[i-1].pose
            wp_curr = waypoints[i].pose

            dist = math.hypot(
                wp_curr.position.x - wp_prev.position.x, 
                wp_curr.position.y - wp_prev.position.y
            )
            if dist < 0.001: continue

            yaw_prev = self.get_yaw(wp_prev.orientation)
            yaw_curr = self.get_yaw(wp_curr.orientation)

            yaw_diff = yaw_curr - yaw_prev
            yaw_diff = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))

            curvature = abs(yaw_diff) / dist
            
            if curvature < self.CURVATURE_THRESHOLD:
                if not is_in_straight:
                    start_index = i
                    is_in_straight = True
            else:
                if is_in_straight:
                    end_index = i
                    if (end_index - start_index) >= self.MIN_SECTION_LENGTH:
                        straight_segments.append((start_index, end_index))
                    is_in_straight = False
        
        return straight_segments

    def find_closest_waypoint(self, curr_pose, waypoints):
        min_dist = float('inf')
        closest_idx = 0
        
        cx = curr_pose.pose.position.x
        cy = curr_pose.pose.position.y

        for i, wp in enumerate(waypoints):
            wx = wp.pose.position.x
            wy = wp.pose.position.y
            dist = (cx - wx)**2 + (cy - wy)**2
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
                
        return closest_idx

    def get_yaw(self, orientation):
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

def main():
    try:
        OvertakeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
