# Overtaking Enhancement - Implementation Specification

**Date:** November 25, 2025
**Status:** Scoped Implementation (time-constrained)
**Framework:** ROS2 Humble, Python

## Scope

Original plan scaled down due to limited time. Focus on functional integration over sophisticated algorithms.

**Package 1:** Hardcoded overtake zones per map (instead of dynamic curvature analysis)
**Package 2:** Simple threshold checks (instead of complex time-benefit calculations)
**Package 3:** Feature freeze, document existing implementation

---

## Message Interfaces

### Package 1 → Package 2

**Topic:** `/overtake_opportunity`
**Message:** `f110_msgs/OvertakeOpportunity`
**Frequency:** 10 Hz

```
std_msgs/Header header
bool is_safe_zone          # True if in straight section
float64 zone_start_s       # Zone start (Frenet s coordinate)
float64 zone_end_s         # Zone end (Frenet s coordinate)
```

---

### Package 3 → Package 2

**Topic:** Confirm with Teammate 3 (likely `/perception/obstacles` or `/predicted_obstacle`)
**Frequency:** 40 Hz

**Required Fields:**
```
float64 predicted_s        # Predicted longitudinal position
float64 predicted_d        # Predicted lateral position (optional)
float64 current_velocity   # Current opponent velocity
float64 acceleration       # Estimated acceleration (optional)
```

**Note:** Use existing message format from Teammate 3's implementation, ensure predicted position field is available.

---

## Integration Points

### Package 2: State Machine Modifications

**File:** `state_machine/state_machine/transitions.py`

**Function:** `SpliniTrailingTransition()`

Add three new checks before returning `StateType.OVERTAKE`:
1. `_check_in_overtake_zone()` - Subscribe to `/overtake_opportunity`
2. `_check_velocity_advantage()` - Compare ego vs opponent velocity
3. `_check_predicted_clearance()` - Use predicted distance from Package 3

**Decision Logic:**
```python
# After existing checks (gb_free, ot_sector, splini_wpts, ofree):
if all([
    state_machine._check_in_overtake_zone(),      # Package 1 input
    state_machine._check_velocity_advantage(),     # Simple velocity diff
    state_machine._check_predicted_clearance()     # Package 3 input
]):
    return StateType.OVERTAKE
```

---

## Shared Parameters

**File:** `stack_master/config/overtaking_params.yaml` (create if needed)

```yaml
# Package 2 thresholds
velocity_threshold: 0.5    # m/s, minimum speed advantage to overtake
safe_margin: 2.0           # meters, minimum predicted clearance

# Package 1 map zones (example structure)
overtake_zones:
  map_name_1: [[start_s1, end_s1], [start_s2, end_s2]]
  map_name_2: [[start_s1, end_s1]]
```

**Note:** Tune these during integration testing.

---

## File Locations

```
Package 1: planner/local_planners/track_section_analyzer/
           - section_analyzer_node.py (new)

Package 2: state_machine/state_machine/
           - enhanced_decision/decision_logic.py (created)
           - transitions.py (modify)
           - state_machine.py (add subscribers)

Package 3: controller/
           - controller_manager.py (existing modifications)
           - pp.py (existing modifications)
```

---

## Testing Protocol

### Phase 1: Individual Verification
```bash
# Verify topics publish correctly
ros2 topic echo /overtake_opportunity
ros2 topic hz /overtake_opportunity
```

### Phase 2: Integration Test
```bash
# Launch full system
ros2 launch stack_master head_to_head_launch.xml racecar_version:=SIM LU_table:=default ctrl_algo:=PP

# Monitor state machine
ros2 topic echo /state_machine

# Check transitions occur
# Expected: GB_TRACK → TRAILING → OVERTAKE (when conditions met)
```

### Phase 3: Behavioral Check
- Verify car attempts overtake when all conditions satisfied
- Check for crashes or unsafe behavior
- Compare lap time to baseline (optional)

---

## Integration Contract

1. **Message formats locked** - Changes require team discussion
2. **Each owner fixes their output** - Don't modify other packages to adapt to your format
3. **Graceful degradation** - Handle missing data (use last known value or safe default)
4. **Backup plan** - If full integration fails, demo packages individually

---

## Critical Synchronization Points

**Before Integration:**
- [ ] Confirm Package 3 topic name and message format
- [ ] Agree on Frenet coordinate frame reference (same global waypoints?)
- [ ] Test message publishing/subscribing independently

**During Integration:**
- [ ] All three packages running simultaneously without crashes
- [ ] State machine receives both `/overtake_opportunity` and opponent prediction
- [ ] State transitions occur (even if behavior is suboptimal)

**For Demo:**
- [ ] Record baseline behavior (without enhancements)
- [ ] Record enhanced system behavior
- [ ] Show ROS topic activity (fallback if car behavior unclear)
