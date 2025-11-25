# Package 2: Enhanced Decision Planner - Implementation

## Overview

Package 2 adds intelligent time-benefit analysis to the ForzaETH state machine's overtaking decisions. Instead of overtaking whenever an opportunity arises, the system now evaluates whether the maneuver actually saves time compared to trailing the opponent.

**Key Feature**: Only overtake when time saved ≥ 0.5 seconds (configurable)

## Message Interface

Package 2 is a pure decision layer that enhances existing state machine logic without publishing new topics.

### Input Topics

```
/car_state/frenet/odom (nav_msgs/Odometry)
└─ Provides: Ego velocity (self.cur_vs from msg.twist.twist.linear.x)
└─ Purpose: Calculate time-overtaking based on our speed

/perception/obstacles (f110_msgs/ObstacleArray)
└─ Provides: Opponent position (s_center, d_center) and velocity (vs)
└─ Purpose: Find closest opponent and calculate time-trailing

/overtake_opportunity (f110_msgs/OvertakeOpportunity) [OPTIONAL - Future]
└─ Provides: Safe zone indication from Package 1
└─ Purpose: Coordinate with curvature-based track analysis
```

###

 Output

**No new topics published** - Package 2 modifies state transitions internally

```mermaid
graph LR
    A[/car_state/frenet/odom] -->|cur_vs| SM[State Machine]
    B[/perception/obstacles] -->|opponent data| SM
    SM -->|enhanced checks| T[Transition Logic]
    T --> D{OVERTAKE?}
```

## Architecture

### Design Pattern: Property-Based Checks

The state machine already uses `@property` decorators for transition conditions. Package 2 follows this existing pattern to maintain code consistency.

**Existing Pattern**:
```python
@property
def _check_gbfree(self) -> bool:
    """Returns True if global track is obstacle-free"""
    # Implementation checks obstacles within horizon
```

**Package 2 Addition**:
```python
@property
def _check_enhanced_time_benefit(self) -> bool:
    """Returns True if overtaking saves time"""
    # Implementation compares trailing vs. overtaking time
```

### Integration Strategy: Additive, Not Destructive

Package 2 adds **one additional AND condition** to the existing overtaking transition logic.

**Before (transitions.py:72)**:
```python
elif (
    not gb_free
    and ot_sector
    and state_machine._check_availability_splini_wpts
    and state_machine._check_ofree
):
    return StateType.OVERTAKE
```

**After (transitions.py:77)**:
```python
elif (
    not gb_free
    and ot_sector
    and state_machine._check_availability_splini_wpts
    and state_machine._check_ofree
    and state_machine._check_enhanced_time_benefit       # Package 2
):
    return StateType.OVERTAKE
```

```mermaid
flowchart TB
    subgraph "Existing State Machine"
        E1[_check_gbfree]
        E2[_check_ot_sector]
        E3[_check_ofree]
    end

    subgraph "Package 2 Addition"
        P1[_check_enhanced_time_benefit]
    end

    E1 & E2 & E3 & P1 --> TR[SpliniTrailingTransition]
    TR --> OT[StateType.OVERTAKE]

    style P1 fill:#4CAF50
```

## Implementation

### Time-Benefit Analysis Formula

**Core Concept**: Compare time spent trailing opponent vs. time spent overtaking

```python
def calculate_time_benefit(ego_velocity, opponent_velocity, distance=10.0):
    maneuver_cost = 2.0  # seconds (lane change overhead)

    time_trailing = distance / opponent_velocity
    time_overtaking = distance / ego_velocity + maneuver_cost

    return time_trailing - time_overtaking  # Positive = saves time
```

**Key Parameters**:
- `distance`: Look-ahead distance for calculation (default 10m)
- `maneuver_cost`: Time penalty for lane change maneuver (2.0 seconds)
- `enhanced_time_benefit_threshold`: Minimum time saved to trigger overtake (0.5s default)

### Example Calculation

**Scenario**: Ego at 5 m/s, Opponent at 3 m/s, Distance 10m

```
Trailing Time:
  10m ÷ 3 m/s = 3.33 seconds

Overtaking Time:
  10m ÷ 5 m/s + 2.0s maneuver = 4.0 seconds

Time Benefit:
  3.33s - 4.0s = -0.67 seconds

Decision:
  -0.67s < 0.5s threshold → ❌ DON'T OVERTAKE
  (Maneuver cost outweighs speed advantage)
```

```mermaid
graph TD
    A[Ego: 5 m/s<br/>Opponent: 3 m/s<br/>Distance: 10m] --> B{Calculate}
    B -->|Trailing| C[10m ÷ 3m/s<br/>= 3.33s]
    B -->|Overtaking| D[10m ÷ 5m/s + 2.0s<br/>= 4.0s]
    C & D --> E[Benefit: 3.33 - 4.0<br/>= -0.67s]
    E --> F[❌ Don't Overtake<br/>Not worth maneuver cost]

    style F fill:#f44336,color:#fff
```

## Code Locations

| Component | File | Line | Purpose |
|-----------|------|------|---------|
| **Decision Logic** | `state_machine/enhanced_decision/decision_logic.py` | 28 | `calculate_time_benefit()` method |
| **Check Method** | `state_machine/state_machine.py` | 395 | `_check_enhanced_time_benefit()` property |
| **Transition Integration** | `state_machine/transitions.py` | 77 | Added condition to `SpliniTrailingTransition()` |
| **Parameters** | `stack_master/config/state_machine_params.yaml` | 30 | Configuration values |
| **Parameter Declarations** | `state_machine/state_machine_params.py` | 216 | Parameter descriptors |

## Configuration

**File**: `stack_master/config/state_machine_params.yaml`

```yaml
# Package 2: Enhanced Decision Planner
enhanced_time_benefit_threshold: 0.5  # [s] Minimum time saving for overtaking
use_safe_zone_check: false            # Enable Package 1 integration (optional)
```

**Tuning Guidelines**:
- **Higher threshold** (e.g., 1.0s): More conservative, only overtake with clear time advantage
- **Lower threshold** (e.g., 0.2s): More aggressive, overtake with small time savings
- **Default 0.5s**: Balanced approach, accounts for maneuver cost uncertainty

## Testing

### Build and Run

```bash
cd ~/ws
colcon build --packages-select state_machine
source install/setup.bash

# Run in simulation (head-to-head mode required)
ros2 launch stack_master head_to_head_launch.xml \
    racecar_version:=SIM \
    LU_table:=default \
    ctrl_algo:=PP
```

### Verification

**Expected Behavior**:
- State machine loads without errors
- `enhanced_time_benefit_threshold` parameter is readable:
  ```bash
  ros2 param get /state_machine enhanced_time_benefit_threshold
  ```
- Overtaking only occurs when time-benefit check passes
- No new topics published (verify with `ros2 topic list`)

**Debug Logging** (optional):
Add to `state_machine.py:427` for visibility:
```python
self.get_logger().info(f"Time benefit: {time_benefit:.2f}s (threshold: {self.params.enhanced_time_benefit_threshold}s)")
```

## Implementation Status

| Feature | Status | Notes |
|---------|--------|-------|
| **Time-Benefit Analysis** | ✅ Implemented | Fully functional, tested in simulation |
| **Dynamic Safety Margin** | ⏳ Deferred | Designed but not deployed (future work) |
| **Package 1 Integration** | 🔌 Ready | Message interface defined, awaiting Package 1 completion |

## Future Work

### Dynamic Safety Margin (Deferred)
**Concept**: Speed-dependent minimum distance requirement
- Formula: `margin = base_margin + k_speed * ego_velocity`
- **Why Deferred**: Time-benefit analysis alone provides sufficient improvement for interim demo
- **Integration Plan**: Add `_check_enhanced_safety_margin()` property when needed

### Package 1 Safe Zone Integration
**Status**: Interface ready, implementation pending
- Message: `f110_msgs/OvertakeOpportunity` already defined
- Topic: `/overtake_opportunity` subscription ready
- **Waiting On**: Package 1's curvature-based track section analyzer
- **Integration**: Add `use_safe_zone_check` parameter to enable when available

## References

- **Original Proposal**: `docs/251123-overtaking-enhancement.md`
- **Implementation Spec**: `docs/251125-implementation-spec.md`
- **Team Coordination**: See `docs/project.md` for Package 1 and Package 3 dependencies
