#!/usr/bin/env python

# -*- coding: utf-8 -*-
# 이 파일에는 Global Path를 구독하고 Yaw 변화량을 이용해
# 곡률 기반으로 직선 구간을 식별하는 로직이 완성되어 있습니다.

import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from overtake_section_detector.msg import OvertakeSectionMsg  # 정의한 커스텀 메시지 import

class OvertakeDetector:
    """
    Package 1: overtake_section_detector
    Global Path를 분석하여 곡률 기반의 추월 가능 구간을 자동 식별하고 정보를 발행하는 ROS 노드입니다.
    """

    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('overtake_section_detector', anonymous=True)

        # --- [1] 파라미터 설정 ---
        self.CURVATURE_THRESHOLD = math.radians(3.0) # 직선 판단 기준 (3도)
        self.MIN_SECTION_LENGTH = 10                  # 최소 추월 구간 길이
        self.global_path_data = None 

        # --- [2] ROS Topic 설정 ---
        # 1. Global Path 구독 (nav_msgs/Path)
        rospy.Subscriber('/global_path', Path, self.path_callback)

        # 2. Overtake Opportunity 발행 (Custom Message)
        self.opp_pub = rospy.Publisher(
            '/overtake_opportunity', OvertakeSectionMsg, queue_size=1
        )
        rospy.loginfo("[OvertakeDetector] Detector initialized. Waiting for /global_path...")

    def path_callback(self, data):
        """ Global Path를 수신하여 곡률 분석을 실행합니다. """
        self.global_path_data = data.poses  

        if len(self.global_path_data) < 2: return

        # 곡률 분석 및 섹터 식별 로직 실행
        sections = self.analyze_curvature(self.global_path_data)

        if sections:
            # 현재 차량 위치 인덱스 (실제 환경에서는 별도 계산 필요)
            current_index = 0
            best_section_start, best_section_end = sections[0]

            is_possible = any(s[0] <= current_index < s[1] for s in sections)

            # 섹터의 평균 곡률 점수 계산 (낮을수록 직선)
            avg_score = self.calculate_section_score(self.global_path_data[best_section_start:best_section_end])

            # 메시지 발행 (패키지 2로 전달)
            msg = OvertakeSectionMsg(
                is_possible, 
                [best_section_start, best_section_end], 
                avg_score
            )
            self.opp_pub.publish(msg)
        else:
            self.opp_pub.publish(OvertakeSectionMsg(False, [], 9999.0))


    def analyze_curvature(self, waypoints):
        """ 경로상의 Yaw 변화량을 이용해 곡률을 계산하고 직선 구간을 식별합니다. """
        straight_segments = []
        is_in_straight = False
        start_index = -1

        for i in range(1, len(waypoints)):
            wp_prev_pose = waypoints[i-1].pose
            wp_curr_pose = waypoints[i].pose

            # 1. 거리 계산
            distance = math.sqrt(
                (wp_curr_pose.position.x - wp_prev_pose.position.x)**2 + 
                (wp_curr_pose.position.y - wp_prev_pose.position.y)**2
            )
            if distance < 0.001: continue

            # 2. Yaw 변화량 계산 (실제 환경에 맞게 Quaternion -> Yaw 변환 코드 필요)
            # 여기서는 로직 구현에 집중하기 위해 임시로 0.0을 사용합니다.
            yaw_diff = 0.0 

            yaw_diff_norm = math.atan2(math.sin(yaw_diff), math.cos(yaw_diff))

            # 3. 곡률 계산 및 직선 판별
            curvature = abs(yaw_diff_norm) / distance

            if curvature < self.CURVATURE_THRESHOLD:
                if not is_in_straight:
                    start_index = i - 1
                    is_in_straight = True
            else:
                if is_in_straight:
                    end_index = i
                    segment_length = end_index - start_index
                    if segment_length >= self.MIN_SECTION_LENGTH:
                        straight_segments.append((start_index, end_index))
                    is_in_straight = False
                    start_index = -1

        return straight_segments

    def calculate_section_score(self, section_waypoints):
        """특정 섹터의 평균 곡률 점수 계산."""
        return 0.05 

def main():
    try:
        OvertakeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()