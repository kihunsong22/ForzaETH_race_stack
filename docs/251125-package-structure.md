# Package 2 Structure Created

**Date:** November 25, 2025
**Status:** Initial Setup Complete

## What Was Created

### 1. Message Definition

**File:** `utilities/libraries/f110_msgs/msg/OvertakeOpportunity.msg`

Simple message for Package 1 to indicate safe overtaking zones:
```
std_msgs/Header header
bool is_safe_zone           # True if in straight section
float64 zone_start_s        # Optional: zone start position
float64 zone_end_s          # Optional: zone end position
```

### 2. Enhanced Decision Module

**Location:** `state_machine/state_machine/enhanced_decision/`

```
enhanced_decision/
├── __init__.py              # Module exports
├── decision_logic.py        # Core decision logic
└── mock_publisher.py        # Mock for Package 1 (testing)
```

### 3. Core Logic (decision_logic.py)

Simple `EnhancedDecisionLogic` class with three methods:

1. **calculate_time_benefit()** - Time saved by overtaking
   - Formula: `time_trailing - time_overtaking`
   - Returns: seconds saved (positive = beneficial)

2. **calculate_safety_margin()** - Dynamic margin based on speed
   - Formula: `base_margin + k_speed * velocity`
   - Returns: required distance in meters

3. **should_overtake()** - Main decision function
   - Checks: safe zone + time benefit + safety margin
   - Returns: True/False

### 4. Mock Publisher

**Purpose:** Enable Package 2 development without waiting for Package 1

**Run with:** `ros2 run state_machine mock_overtake_publisher`

Publishes `/overtake_opportunity` at 10 Hz with `is_safe_zone=True`

## Next Steps

### To Build and Test

```bash
# Inside Docker container
cd ~/ws
colcon build --packages-select f110_msgs state_machine
source install/setup.bash

# Test mock publisher
ros2 run state_machine mock_overtake_publisher

# In another terminal, verify message
ros2 topic echo /overtake_opportunity
```

### Integration Steps (Next Phase)

1. Subscribe to `/overtake_opportunity` in state_machine.py
2. Subscribe to opponent data from `/perception/obstacles`
3. Add methods `_check_time_benefit()` and `_check_dynamic_safety()`
4. Modify transition logic in transitions.py

## Parameters to Tune Later

- `time_benefit_threshold`: 0.5 seconds (default)
- `base_safety_margin`: 1.0 meters (default)
- `speed_margin_factor`: 0.1 m per m/s (default)
- `maneuver_cost`: 2.0 seconds (hardcoded, to tune)

## Design Decisions

**Keep it simple:**
- Single file for core logic (decision_logic.py)
- No complex classes or inheritance
- Straightforward calculations
- Easy to test and tune

**Mock data first:**
- Enables parallel development
- Teammates can use mock immediately
- Switch to real data when Package 1 ready
