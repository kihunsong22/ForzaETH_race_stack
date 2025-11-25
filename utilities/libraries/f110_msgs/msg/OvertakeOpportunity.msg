# Message for Track Section Analyzer (Package 1) to indicate safe overtaking zones

std_msgs/Header header

# Simple flag: true if currently in a safe overtaking zone (straight section)
bool is_safe_zone

# Optional: position range of the safe zone in Frenet s-coordinate
# Can be -1 if not applicable
float64 zone_start_s
float64 zone_end_s
