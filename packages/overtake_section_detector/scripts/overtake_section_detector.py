#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from overtake_section_detector.msg import OvertakeSectionMsg 

class OvertakeDetector(Node):
    def __init__(self):
        super().__init__('overtake_section_detector')

        self.CURVATURE_THRESHOLD = 0.04
        self.MIN_SECTION_LENGTH = 15
        
        self.global_path_poses = []
        self.straight_sections = [] 
        self.current_pose = None
        
        self.pose_topic = '/current_pose' 
        self.path_topic = '/global_path'

        qos_profile = QoSProfile(depth=10)

        self.create_subscription(
            Path, 
            self.path_topic, 
            self.path_callback, 
            qos_profile
        )
        self.create_subscription(
            PoseStamped, 
            self.pose_topic, 
            self.pose_callback, 
            qos_profile
        )

        self.opp_pub = self.create_publisher(
            OvertakeSectionMsg, 
            '/overtake_opportunity', 
            qos_profile
        )

        self.create_timer(0.1, self.timer_callback)

    def path_callback(self, msg):
        self.global_path_poses = msg.poses
        if len(self.global_path_poses) < 2:
            return
        
        self.straight_sections = self.analyze_curvature(self.global_path_poses)

    def pose_callback(self, msg):
        self.current_pose = msg

    def timer_callback(self):
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

    def get_yaw(self, q):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        
        sinp = 2 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = OvertakeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
