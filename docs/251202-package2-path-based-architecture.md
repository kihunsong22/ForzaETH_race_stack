# Package 2: Path-Based Time-Benefit Analysis - Architecture

## Abstract

Package 2 enhances the ForzaETH state machine's overtaking decisions by comparing the **time required to traverse different paths**. Instead of using arbitrary maneuver cost constants, the system calculates actual path distances from the Spliner local planner and converts them to time using vehicle velocities. This approach is:
- **Physics-based**: Pure kinematics (distance/velocity = time)
- **Scale-appropriate**: No oversized time penalties for F1/10 racing
- **Defensible**: Uses real trajectory data from existing planner

**Key Innovation**: Geometric path comparison with velocity-aware time conversion - no arbitrary constants.

---

## System Architecture

### Component Interaction

```
┌─────────────────────────────────────────────────────────────┐
│                      State Machine                          │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  _check_enhanced_time_benefit() [NEW]                │  │
│  │                                                       │  │
│  │  Inputs:                                             │  │
│  │   - self.params.v (ego velocity)                     │  │
│  │   - self.params.opponent_velocity                    │  │
│  │   - self.avoidance_wpnts (Spliner trajectory)        │  │
│  │   - self.params.enhanced_lookahead_distance          │  │
│  │                                                       │  │
│  │  Calls:                                              │  │
│  │   - EnhancedDecisionLogic.calculate_time_benefit()   │  │
│  │                                                       │  │
│  │  Returns:                                            │  │
│  │   - bool (True = overtaking saves time)              │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
│  Integrates with existing transition logic:                │
│   if (ot_sector AND ofree AND splini_wpts AND              │
│       _check_enhanced_time_benefit):  ← NEW                │
│       → StateType.OVERTAKE                                 │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│            EnhancedDecisionLogic (decision_logic.py)        │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  calculate_arc_length(waypoints) [STATIC]            │  │
│  │  - Sums Euclidean distances between waypoint pairs   │  │
│  │  - Returns total path length (meters)                │  │
│  └──────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  calculate_time_benefit_from_paths()                 │  │
│  │                                                       │  │
│  │  Inputs:                                             │  │
│  │   - ego_velocity (m/s)                               │  │
│  │   - opponent_velocity (m/s)                          │  │
│  │   - following_distance (m) - racing line lookahead   │  │
│  │   - overtaking_waypoints - Spliner trajectory        │  │
│  │                                                       │  │
│  │  Logic:                                              │  │
│  │   d_overtake = calculate_arc_length(overtaking_wpts) │  │
│  │   time_follow = d_follow / v_opponent                │  │
│  │   time_overtake = d_overtake / v_ego                 │  │
│  │   benefit = time_follow - time_overtake              │  │
│  │                                                       │  │
│  │  Returns:                                            │  │
│  │   - Time benefit (seconds, positive = faster)        │  │
│  └──────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

---

## Message Flow

### Input Topics

```
Topic: /car_state/frenet/odom
Type:  nav_msgs/Odometry
Data:  msg.twist.twist.linear.x
Use:   → self.params.v (ego velocity)
       → Used in time_overtake = d_overtake / v_ego

Topic: /perception/obstacles
Type:  f110_msgs/ObstacleArray
Data:  closest_obstacle.vs
Use:   → self.params.opponent_velocity
       → Used in time_follow = d_follow / v_opponent

Topic: /planner/avoidance/otwpnts
Type:  nav_msgs/Path
Data:  waypoints[] (PoseStamped array)
Use:   → self.avoidance_wpnts
       → Used in d_overtake = calculate_arc_length(waypoints)
```

### Internal Data Flow

```
State Machine Update Cycle (every iteration):

1. Read sensor data
   └─→ /car_state/frenet/odom → self.params.v
   └─→ /perception/obstacles → self.params.opponent_velocity

2. Spliner generates avoidance trajectory
   └─→ /planner/avoidance/otwpnts → self.avoidance_wpnts

3. Transition check: SpliniTrailingTransition()
   ├─→ _check_ot_sector → bool
   ├─→ _check_ofree → bool
   ├─→ _check_availability_splini_wpts → bool
   └─→ _check_enhanced_time_benefit → bool  [NEW]
       │
       ├─→ EnhancedDecisionLogic.calculate_time_benefit_from_paths()
       │   │
       │   ├─→ calculate_arc_length(self.avoidance_wpnts) → d_overtake
       │   ├─→ d_follow = self.params.enhanced_lookahead_distance
       │   ├─→ time_follow = d_follow / opponent_velocity
       │   ├─→ time_overtake = d_overtake / ego_velocity
       │   └─→ return time_follow - time_overtake
       │
       └─→ (benefit >= threshold) ? True : False

4. If all checks pass:
   └─→ Transition to StateType.OVERTAKE
```

### No Output Topics

Package 2 is a **decision layer only** - no new topics published.
- Integrates silently into existing state machine transitions
- Changes are internal to state machine logic

---

## Configuration Parameters

**File**: `stack_master/config/state_machine_params.yaml`

```yaml
# Package 2: Enhanced Decision Planner (Path-Based)
enhanced_time_benefit_threshold: 0.5   # [s] Minimum time saving to overtake
enhanced_lookahead_distance: 10.0      # [m] Following path distance for comparison
```

**Parameters**:

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `enhanced_time_benefit_threshold` | float | 0.5 | 0.0-5.0 | Minimum time benefit (seconds) required to trigger overtake |
| `enhanced_lookahead_distance` | float | 10.0 | 5.0-30.0 | Distance (meters) along racing line for trailing time calculation |

**Removed (from previous design)**:
- ~~`enhanced_base_maneuver_cost`~~ - Arbitrary constant not appropriate for F1/10 scale
- ~~`enhanced_velocity_cost_factor`~~ - Velocity-dependence not justified by physics at this scale

---

## Key Formulas

### Arc Length Calculation
```python
def calculate_arc_length(waypoints):
    total = 0.0
    for i in range(len(waypoints) - 1):
        dx = waypoints[i+1].x - waypoints[i].x
        dy = waypoints[i+1].y - waypoints[i].y
        total += sqrt(dx² + dy²)
    return total
```

### Time-Benefit Calculation
```python
# Distances
d_follow = lookahead_distance        # e.g., 10m along racing line
d_overtake = arc_length(avoidance_waypoints)  # from Spliner

# Times (pure kinematics)
time_follow = d_follow / v_opponent
time_overtake = d_overtake / v_ego

# Decision
time_benefit = time_follow - time_overtake
should_overtake = (time_benefit >= threshold)
```

**Physical Interpretation**:
- Overtaking path is geometrically **longer** (lateral deviation)
- But we traverse it at **higher velocity** (ego faster than opponent)
- Net effect: Compare actual times, not just distances

---

## Example Scenarios

### Scenario 1: Should Overtake
```
Inputs:
  - ego_velocity = 6.0 m/s
  - opponent_velocity = 3.0 m/s
  - d_follow = 10.0 m (racing line)
  - d_overtake = 10.5 m (Spliner path, slightly longer)

Calculation:
  time_follow = 10.0 / 3.0 = 3.33 seconds
  time_overtake = 10.5 / 6.0 = 1.75 seconds
  time_benefit = 3.33 - 1.75 = 1.58 seconds

Decision:
  1.58s >= 0.5s threshold → ✅ OVERTAKE
  (Velocity advantage overcomes longer path)
```

### Scenario 2: Should NOT Overtake
```
Inputs:
  - ego_velocity = 3.5 m/s
  - opponent_velocity = 3.0 m/s
  - d_follow = 10.0 m
  - d_overtake = 10.5 m

Calculation:
  time_follow = 10.0 / 3.0 = 3.33 seconds
  time_overtake = 10.5 / 3.5 = 3.00 seconds
  time_benefit = 3.33 - 3.00 = 0.33 seconds

Decision:
  0.33s < 0.5s threshold → ❌ DON'T OVERTAKE
  (Small velocity advantage insufficient)
```

### Scenario 3: Path Much Longer
```
Inputs:
  - ego_velocity = 5.0 m/s
  - opponent_velocity = 4.0 m/s
  - d_follow = 10.0 m
  - d_overtake = 13.0 m (tight corner, long detour)

Calculation:
  time_follow = 10.0 / 4.0 = 2.50 seconds
  time_overtake = 13.0 / 5.0 = 2.60 seconds
  time_benefit = 2.50 - 2.60 = -0.10 seconds

Decision:
  -0.10s < 0.5s threshold → ❌ DON'T OVERTAKE
  (Detour path too long, negates speed advantage)
```

---

## Defense for Professor

**Question**: "How do you decide when to overtake?"

**Answer**:
> "We compare the time required to traverse two paths. The first path is staying on the racing line behind the opponent - we know the distance from our lookahead horizon and the opponent's velocity. The second path is the overtaking trajectory computed by our local planner (Spliner), which generates collision-free paths around obstacles. We calculate the arc length of this trajectory by summing the Euclidean distances between waypoints. Both distances are converted to time using basic kinematics: time equals distance divided by velocity. The overtaking path is geometrically longer due to lateral deviation, but we traverse it at higher velocity since we're faster than the opponent. We only overtake if the time benefit exceeds our threshold. This is pure physics without arbitrary constants - appropriate for the F1/10 scale where maneuvers happen in seconds."

**Follow-up**: "Why no maneuver cost overhead?"

**Answer**:
> "F1/10 tracks are small - an entire overtaking maneuver takes 1-2 seconds. Adding a 1-2 second constant would dominate the calculation unrealistically. The path length difference already captures the geometric cost of lane changing. Our Spliner trajectory accounts for the actual curvature and distance traveled during the maneuver. Any additional 'decision latency' overhead would be on the order of 0.1-0.2 seconds, which is negligible compared to the path traversal time and falls within our time-benefit threshold margin."

---

## Implementation Files

| Component | File Path | Purpose |
|-----------|-----------|---------|
| **Decision Logic** | `state_machine/state_machine/enhanced_decision/decision_logic.py` | Arc length calculation + time-benefit method |
| **State Machine Check** | `state_machine/state_machine/state_machine.py` | `_check_enhanced_time_benefit()` property |
| **Transition Integration** | `state_machine/state_machine/transitions.py` | Add check to `SpliniTrailingTransition()` |
| **Configuration** | `stack_master/config/state_machine_params.yaml` | Parameters (threshold, lookahead) |
| **Parameter Declarations** | `state_machine/state_machine/state_machine_params.py` | ROS2 parameter descriptors |

---

## Testing Plan

1. **Unit Test**: `calculate_arc_length()` with mock waypoints
2. **Unit Test**: `calculate_time_benefit_from_paths()` with known velocities
3. **Integration Test**: Launch in simulation, verify state transitions
4. **Verification**: Monitor `/state_machine` topic for OVERTAKE triggers
5. **Parameter Sweep**: Test different thresholds (0.2s, 0.5s, 1.0s)

---

## Scalability

**Current Implementation**: Path from Spliner (full collision-free trajectory)

**Future Enhancements** (if needed):
- Account for path curvature (tighter turns → lower speeds)
- Integrate opponent prediction (if opponent slows down, recalculate)
- Multi-step lookahead (compare benefit over full lap)

**Simplifications** (if needed):
- Approximate overtaking path as circular arc (geometric model)
- Use fixed lateral offset instead of Spliner (d_overtake ≈ sqrt(d_follow² + offset²))

The path-based approach scales in both directions without changing the core formula.
