"""
Enhanced Decision Logic for Package 2

Simple, focused implementation of:
1. Time-benefit analysis
2. Dynamic safety margins
"""

class EnhancedDecisionLogic:
    """
    Evaluates whether overtaking is worthwhile and safe
    """

    def __init__(self,
                 time_benefit_threshold: float = 0.5,
                 base_safety_margin: float = 1.0,
                 speed_margin_factor: float = 0.1):
        """
        Args:
            time_benefit_threshold: Minimum time saving (seconds) to overtake
            base_safety_margin: Minimum distance to opponent (meters)
            speed_margin_factor: Additional margin per m/s of velocity
        """
        self.time_threshold = time_benefit_threshold
        self.base_margin = base_safety_margin
        self.k_speed = speed_margin_factor

    def calculate_time_benefit(self,
                               ego_velocity: float,
                               opponent_velocity: float,
                               distance: float = 10.0) -> float:
        """
        Calculate time benefit of overtaking vs. trailing

        Core Formula:
            time_benefit = time_trailing - time_overtaking
            where:
            - time_trailing = distance / opponent_velocity
            - time_overtaking = distance / ego_velocity + maneuver_cost (2.0s)

        Args:
            ego_velocity: Our current speed (m/s)
            opponent_velocity: Opponent's current speed (m/s)
            distance: Look-ahead distance for calculation (default 10m)

        Returns:
            float: Time benefit in seconds
                   Positive → Overtaking saves time
                   Negative → Trailing is faster (avoid overtaking)

        Example:
            >>> calc = EnhancedDecisionLogic(time_benefit_threshold=0.5)
            >>> calc.calculate_time_benefit(5.0, 3.0, 10.0)
            -0.67  # Don't overtake - not worth the 2s maneuver cost
        """
        if ego_velocity <= 0 or opponent_velocity <= 0:
            return -999.0  # Invalid

        # Assume 2 second maneuver cost (tune later)
        maneuver_cost = 2.0

        time_trailing = distance / opponent_velocity
        time_overtaking = distance / ego_velocity + maneuver_cost

        return time_trailing - time_overtaking

    def calculate_safety_margin(self, ego_velocity: float) -> float:
        """
        Dynamic safety margin based on speed

        Formula: margin = base + k_speed * velocity

        Args:
            ego_velocity: Our speed (m/s)

        Returns:
            Required safety margin in meters
        """
        return self.base_margin + self.k_speed * ego_velocity

    def should_overtake(self,
                       ego_velocity: float,
                       opponent_velocity: float,
                       distance_to_opponent: float,
                       in_safe_zone: bool) -> bool:
        """
        Main decision function: Should we overtake?

        Conditions:
        1. In safe overtaking zone (from Package 1)
        2. Time benefit exceeds threshold
        3. Distance exceeds dynamic safety margin

        Args:
            ego_velocity: Our speed (m/s)
            opponent_velocity: Opponent speed (m/s)
            distance_to_opponent: Distance to opponent (m)
            in_safe_zone: True if Package 1 says we're in straight section

        Returns:
            True if all conditions met
        """
        # Condition 1: Must be in safe zone
        if not in_safe_zone:
            return False

        # Condition 2: Time benefit
        benefit = self.calculate_time_benefit(ego_velocity, opponent_velocity)
        if benefit < self.time_threshold:
            return False

        # Condition 3: Safety margin
        required_margin = self.calculate_safety_margin(ego_velocity)
        if distance_to_opponent < required_margin:
            return False

        return True
