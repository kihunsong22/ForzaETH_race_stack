"""
Enhanced Decision Logic for Package 2

Path-based time-benefit analysis:
- Compares time to traverse racing line vs. overtaking trajectory
- Uses actual Spliner path distances (no arbitrary constants)
- Pure physics: time = distance / velocity
"""

import numpy as np


class EnhancedDecisionLogic:
    """
    Evaluates whether overtaking is worthwhile and safe
    """

    def __init__(self,
                 time_benefit_threshold: float = 0.5,
                 lookahead_distance: float = 10.0,
                 base_safety_margin: float = 1.0,
                 speed_margin_factor: float = 0.1):
        """
        Args:
            time_benefit_threshold: Minimum time saving (seconds) to overtake
            lookahead_distance: Look-ahead distance for calculation (meters)
            base_safety_margin: Minimum distance to opponent (meters)
            speed_margin_factor: Additional margin per m/s of velocity
        """
        self.time_threshold = time_benefit_threshold
        self.lookahead_distance = lookahead_distance
        self.base_margin = base_safety_margin
        self.k_speed = speed_margin_factor

    @staticmethod
    def calculate_arc_length(waypoints) -> float:
        """
        Calculate total path length from waypoint sequence

        Args:
            waypoints: List/array of waypoints with .x and .y attributes (or .pose.pose.position)

        Returns:
            Total arc length in meters
        """
        if not waypoints or len(waypoints) < 2:
            return 0.0

        total_distance = 0.0
        for i in range(len(waypoints) - 1):
            # Handle different waypoint formats
            if hasattr(waypoints[i], 'pose'):
                # nav_msgs/Path format
                x1, y1 = waypoints[i].pose.pose.position.x, waypoints[i].pose.pose.position.y
                x2, y2 = waypoints[i+1].pose.pose.position.x, waypoints[i+1].pose.pose.position.y
            else:
                # Direct waypoint format
                x1, y1 = waypoints[i].x, waypoints[i].y
                x2, y2 = waypoints[i+1].x, waypoints[i+1].y

            dx = x2 - x1
            dy = y2 - y1
            total_distance += np.sqrt(dx**2 + dy**2)

        return total_distance

    def calculate_time_benefit_from_paths(self,
                                          ego_velocity: float,
                                          opponent_velocity: float,
                                          following_distance: float,
                                          overtaking_waypoints) -> float:
        """
        Calculate time benefit using actual path distances

        Pure physics - NO arbitrary maneuver cost constants.

        Formula:
            time_benefit = (d_follow / v_opp) - (d_overtake / v_ego)

        The overtaking path is slightly longer geometrically, but we
        traverse it at higher velocity (ego faster than opponent).

        Args:
            ego_velocity: Our speed (m/s)
            opponent_velocity: Opponent speed (m/s)
            following_distance: Distance along racing line (m)
            overtaking_waypoints: Waypoints from Spliner (/planner/avoidance/otwpnts)

        Returns:
            Time benefit in seconds
            Positive = overtaking is faster
            Negative = trailing is faster

        Example:
            ego=6.0 m/s, opp=3.0 m/s, d_follow=10.0m, d_overtake=10.5m
            time_follow = 10.0/3.0 = 3.33s
            time_overtake = 10.5/6.0 = 1.75s
            benefit = 3.33 - 1.75 = 1.58s → OVERTAKE
        """
        if ego_velocity <= 0 or opponent_velocity <= 0:
            return -999.0  # Invalid velocities

        # Calculate overtaking path distance from Spliner
        d_overtake = self.calculate_arc_length(overtaking_waypoints)
        if d_overtake == 0:
            d_overtake = following_distance  # Fallback if path empty

        # Pure time comparison - NO maneuver cost
        time_follow = following_distance / opponent_velocity
        time_overtake = d_overtake / ego_velocity

        return time_follow - time_overtake

    def calculate_safety_margin(self, ego_velocity: float) -> float:
        """
        Calculate dynamic safety margin based on speed

        Risk-aware: Higher speeds require larger safety margins due to:
        - Longer braking distances
        - Reduced reaction time
        - Higher impact severity

        Formula:
            margin = base_margin + k_speed * ego_velocity

        Args:
            ego_velocity: Our speed (m/s)

        Returns:
            Required safety margin in meters

        Example:
            At 3 m/s: 1.0 + 0.1*3 = 1.3m
            At 5 m/s: 1.0 + 0.1*5 = 1.5m
            At 8 m/s: 1.0 + 0.1*8 = 1.8m
        """
        return self.base_margin + self.k_speed * ego_velocity
