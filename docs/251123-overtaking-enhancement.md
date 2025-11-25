# Context-Aware Autonomous Overtaking System

**Date:** November 23, 2025
**Status:** Original Proposal - Subject to Change

## Objective

Enhance ForzaETH race stack's overtaking capabilities through intelligent, context-aware decision-making to reduce lap times.

## Team

- Teammate 1 - Package 1: Track Section Analyzer
- Teammate 2 (Lead) - Package 2: Enhanced Decision Planner
- Teammate 3 - Package 3: Perception & State Estimator

## Architecture Overview

Three new ROS2 packages (Python/rospy):

### Package 1: Track Section Analyzer
**Owner:** Teammate 1
**Location:** `planner/local_planners/track_section_analyzer/`

- **Goal:** Automate overtaking zone identification via curvature analysis
- **Input:** Global path waypoints (x, y, orientation)
- **Output:** `/overtake_opportunity` topic
- **Logic:** `curvature = Δyaw / Δs`, identify straight sections (low curvature) as safe zones

### Package 2: Enhanced Decision Planner
**Owner:** Teammate 2 (Lead)
**Location:** `state_machine/enhanced_decision/`

- **Goal:** Improve binary TRAILING/OVERTAKE logic with time-benefit analysis and risk assessment
- **Input:** `/overtake_opportunity` + opponent state from Package 3
- **Output:** State machine transitions (GBTRACK → TRAILING → OVERTAKE)
- **Features:**
  - Time benefit: `Δt = time_saved - time_cost_of_maneuver`
  - Dynamic safety margins: `margin = base_margin + k_speed * velocity`

### Package 3: Perception & State Estimator
**Owner:** Teammate 3
**Location:** `perception/trajectory_prediction/`

- **Goal:** Predict opponent trajectory for proactive control
- **Input:** `/perception/obstacles`
- **Method:** Constant acceleration model
  - Acceleration: `a = (v_current - v_previous) / dt`
  - Prediction: `s_future = s_0 + v*t + 0.5*a*t^2`
- **Output:** Enhanced obstacle state with trajectory prediction

## Message Flow

```
Global Path → Package 1 → /overtake_opportunity → Package 2
                          /perception/obstacles (enhanced by Package 3) → Package 2
                                                    Package 2 → State Machine → Controller
```

## Key Parameters (To Be Tuned)

### Package 1
- `curvature_threshold`: Maximum curvature for safe overtaking zone
- `min_section_length`: Minimum straight section length

### Package 2
- `time_benefit_threshold`: Minimum time saving to trigger overtake
- `base_safety_margin`: Base distance margin (meters)
- `k_speed`: Speed-dependent margin scaling factor

### Package 3
- `prediction_horizon`: Prediction time horizon (seconds)
- `acceleration_filter_alpha`: Smoothing factor for acceleration estimation

## Development Strategy

**Parallel Development with Mock Data:**
- Phase 1: Create mock publishers for Packages 1 & 3
- Phase 2: Implement real logic, integrate incrementally
- Phase 3: Full system testing and parameter tuning

**Integration Points:**
- Package 1 mock: Hardcoded straight sections
- Package 3 mock: Constant velocity predictions
- Enables Package 2 development without waiting

## Testing Plan

1. **Unit Testing:** Each package with synthetic data
2. **Integration Testing:** Message flow verification
3. **Performance Testing:** Lap time comparison (baseline vs. enhanced)

## Technical Decisions

**Scope Simplifications:**
- Dropped: Monte Carlo simulations, covariance propagation, MPC constraint modifications
- Kept: Basic time-benefit calculations, distance-based safety margins, trajectory prediction module
- Rationale: Timeline constraints + team new to ROS2

**Implementation Choices:**
- Python/ROSPY only (matches educational fork)
- Curvature via yaw angle changes (simpler than arc fitting)
- Constant acceleration model (balance of simplicity/accuracy)

## Repository Context

**Base:** Professor's educational fork (Python-converted from C++ original)
- Removed ~60% of advanced modules from original ForzaETH
- Added educational infrastructure (f1tenth_system, slam_toolbox, particle_filter)
- All packages converted from C++ CMake to Python setuptools

## Known Limitations

- Curvature-only analysis (ignores track width variations)
- Constant acceleration may not capture strategy changes
- Single opponent assumption
- No multi-opponent coordination

## Future Enhancements

- Monte Carlo risk assessment
- ML-based opponent behavior prediction
- Multi-opponent scenarios
- Track-specific tuning profiles
