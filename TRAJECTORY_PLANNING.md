# Trajectory Planning Module for ctrl_1

## Overview

This module adds real-time trajectory planning with obstacle avoidance to the ctrl_1 Flair UAV controller. It implements the full pipeline:

1. **3D Path Search** (JPS/A*) on an occupancy grid
2. **Safe Flight Corridor** construction (convex polyhedra around the path)
3. **Minimum-Snap Trajectory Optimization** (7th-degree piecewise polynomials)
4. **Receding Horizon Replanning** at configurable rate
5. **Dynamic Obstacle Tracking** via OptiTrack VRPN with velocity estimation

The module outputs position, velocity, acceleration, and jerk as functions of time — directly compatible with the existing `Sliding_pos` controller.

## Architecture

```
Ground Station (Flair GCS)              UAV Onboard (ARM)
┌──────────────────────────┐    TCP    ┌──────────────────────────────┐
│ [Tab: Setup trajectory]  │◄──────►│ TrajectoryManager              │
│  Planner mode            │          │  ├── OccupancyGrid3D          │
│  Safety margin           │          │  ├── JPS3D path search        │
│  Max velocity / accel    │          │  ├── SafeFlightCorridor       │
│  Grid resolution         │          │  ├── MinSnap QP solver        │
│  Replan period           │          │  ├── VRPN obstacle tracking   │
│  Waypoints WP1..WP5     │          │  └── Receding horizon replan  │
│  [Plan] [Execute] [Stop] │          │                                │
│  Status: Executing       │          │  pos_reference() case 2:      │
└──────────────────────────┘          │    → pos, vel, acc, jerk      │
                                      │    → Sliding_pos controller   │
                                      └──────────────────────────────┘
```

## File Structure

```
uav/src/
├── trajectory_planner/           # Core library (header-only, Eigen-only)
│   ├── trajectory.hpp            # Piece<D> + Trajectory<D> classes
│   ├── minsnap_solver.hpp        # Minimum-snap QP solver
│   ├── occupancy_grid.hpp        # 3D occupancy grid
│   ├── jps3d.hpp                 # Jump Point Search / A* in 3D
│   ├── convex_decomp.hpp         # Safe Flight Corridor (AABB expansion)
│   └── trajectory_planner.hpp    # Top-level planner (pipeline orchestrator)
├── TrajectoryManager.h           # Flair integration: GUI + lifecycle
├── TrajectoryManager.cpp         # Flair integration: implementation
├── ctrl1.h                       # Modified: added traj_manager_ member
└── ctrl1.cpp                     # Modified: wired into pos_reference()
```

## How to Use

### 1. On the Ground Station

1. Select the **"Position"** tab → **"Reference position"** sub-tab
2. Set **"Select behavior"** to **"Trajectory"**
3. In the **"Setup trajectory"** group:
   - Set **Safety margin** (default: 0.30 m)
   - Set **Max velocity** (default: 1.5 m/s)
   - Set **Max accel** (default: 3.0 m/s²)
   - Set **Grid resolution** (default: 0.10 m)
   - Set **Replan period** (default: 2.0 s)
   - Set **Waypoints (N)** to the number of waypoints (1-5)
   - Fill in **WP1** through **WPN** with target positions (x, y, z in meters)
   - Set **Obstacles (N)** to the number of OptiTrack obstacle rigid bodies
4. Click **"Plan"** → status shows "Planned OK" or error
5. Click **"Execute"** → UAV begins tracking the trajectory
6. Click **"Stop"** at any time to halt and hold position

### 2. OptiTrack Setup for Obstacles

- In Motive, create rigid bodies named `obstacle1`, `obstacle2`, etc.
- The VRPN client in ctrl_1 will automatically track them
- Set **Obstacles (N)** in the GUI to match the number of active obstacles
- Obstacles are tracked in real-time and predicted using constant-velocity model

### 3. Receding Horizon

- When executing, the planner automatically replans at the configured **Replan period**
- Each replan uses the UAV's current position as the new start
- Dynamic obstacles are predicted forward based on their current velocity
- If replanning fails, the old trajectory continues executing

## Building

The trajectory planner library is header-only — no changes to the build system beyond adding `TrajectoryManager.cpp` to `SRC_FILES` in `CMakeLists.txt` (already done).

Requirements:
- Eigen 3.3.x or 3.4.x (already configured in the project)
- C++11 (GCC 4.9 compatible)
- Flair SDK

```bash
cmake --preset flair-cross-compile-core2_64
cmake --build --preset default-core2_64
```

## Technical Details

### Min-Snap Trajectory

The trajectory is a piecewise 7th-degree polynomial optimized to minimize the integral of squared snap (4th derivative of position). This produces the smoothest possible trajectory through the waypoints while respecting:

- Position constraints at each waypoint
- Zero velocity, acceleration, and jerk at start and end
- Continuity of derivatives up to 7th order at segment junctions

The polynomial at time `t` gives:
- `p(t)` — desired position
- `p'(t)` — desired velocity
- `p''(t)` — desired acceleration
- `p'''(t)` — desired jerk

These are exactly the outputs required by `Sliding_pos::SetValues()`.

### Obstacle Handling

Static obstacles are represented as spheres in the occupancy grid. Dynamic obstacles use:

1. **Velocity estimation**: Exponential low-pass filter on numerical position differentiation (τ = 0.1 s)
2. **Prediction**: Constant-velocity model sampled at 5 time steps over the planning horizon
3. **Inflation**: Future obstacle positions are inflated by increasing amounts to account for prediction uncertainty

### Thread Safety

All planning is synchronous within Flair's single-threaded control loop. Planning takes 50-300 ms (acceptable as the UAV continues flying the previous trajectory during this time). No mutexes or separate threads are required.

## Limitations and Future Work

- The current waypoint mode does not guarantee collision-free trajectories (the min-snap QP does not enforce corridor constraints). The planner warns if collisions are detected.
- The corridor mode (SFC + constrained QP) is implemented in the library but not yet exposed in the GUI.
- TOPP-RA time-optimal reparametrization can be added as a post-processing step.
- GCOPTER/MINCO extraction can replace the min-snap solver for superior trajectory quality.
