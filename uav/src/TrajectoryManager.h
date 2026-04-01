// TrajectoryManager.h
//
// Manages the trajectory planning lifecycle for ctrl_1 (Flair UAV controller).
// Handles: GUI configuration, obstacle VRPN tracking, replanning, trajectory evaluation.
//
// Phases 2-5 of the trajectory integration roadmap.
//
// C++11 / GCC 4.9 compatible — no C++14/17 features.
// Flair SDK + Eigen 3.3.x/3.4.x.

#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include <string>
#include <vector>

// Eigen
#include <Eigen/Dense>

// Flair core
#include <Vector3D.h>    // flair::core::Vector3Df

// Trajectory planner
#include "trajectory_planner/trajectory.hpp"
#include "trajectory_planner/occupancy_grid.hpp"

// Forward declarations — Flair GUI
namespace flair {
    namespace gui {
        class GroupBox;
        class DoubleSpinBox;
        class PushButton;
        class ComboBox;
        class Label;
        class Tab;
        class TabWidget;
    }
    namespace sensor {
        class VrpnClient;
    }
    namespace meta {
        class MetaVrpnObject;
    }
}

// ============================================================================
// TrajectoryManager
// ============================================================================

class TrajectoryManager {
public:
    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------
    enum class State {
        IDLE,        ///< No trajectory, waiting for Plan
        PLANNING,    ///< Plan requested, computing (inline/synchronous)
        PLANNED,     ///< Trajectory computed, waiting for Execute
        EXECUTING,   ///< Tracking active trajectory
        REPLANNING   ///< Recomputing trajectory while executing (inline)
    };

    // -----------------------------------------------------------------------
    // Per-obstacle data (VRPN + state estimation)
    // -----------------------------------------------------------------------
    struct ObstacleState {
        flair::meta::MetaVrpnObject* vrpn;   ///< Flair VRPN object
        Eigen::Vector3d position;            ///< Current position (m)
        Eigen::Vector3d velocity;            ///< Estimated velocity (m/s), low-pass filtered
        Eigen::Vector3d prev_position;       ///< Previous position for differentiation
        double prev_time;                    ///< Time of previous update (s)
        bool initialized;                    ///< Whether first measurement received

        ObstacleState()
            : vrpn(0)
            , position(Eigen::Vector3d::Zero())
            , velocity(Eigen::Vector3d::Zero())
            , prev_position(Eigen::Vector3d::Zero())
            , prev_time(0.0)
            , initialized(false)
        {}
    };

    // -----------------------------------------------------------------------
    // Constructor / Destructor
    // -----------------------------------------------------------------------

    /// @param parent   GroupBox in which all trajectory GUI widgets are placed.
    ///                 Typically the "Setup trajectory" GroupBox from ctrl1.
    explicit TrajectoryManager(flair::gui::GroupBox* parent);

    ~TrajectoryManager();

    // -----------------------------------------------------------------------
    // Initialisation
    // -----------------------------------------------------------------------

    /// Add a VRPN obstacle tracker. Call once per obstacle in ctrl1 constructor
    /// after vrpnclient->Start(). @p name must match the Motive rigid body name.
    void addObstacleVrpn(const std::string& name,
                         flair::sensor::VrpnClient* client);

    // -----------------------------------------------------------------------
    // Real-time interface (called every control loop iteration)
    // -----------------------------------------------------------------------

    /// Called every control loop. Checks buttons, updates obstacles, handles
    /// replanning timer. @p uav_pos is the current UAV position in world frame.
    void update(float t_actual, const flair::core::Vector3Df& uav_pos);

    /// Evaluate the active trajectory at global time @p t_actual.
    ///
    /// @param[out] xid     Desired position
    /// @param[out] xidp    Desired velocity
    /// @param[out] xidpp   Desired acceleration
    /// @param[out] xidppp  Desired jerk
    /// @return true if trajectory is active and outputs have been filled;
    ///         false if not executing (caller should fall back to regulation).
    bool evaluate(float t_actual,
                  flair::core::Vector3Df& xid,
                  flair::core::Vector3Df& xidp,
                  flair::core::Vector3Df& xidpp,
                  flair::core::Vector3Df& xidppp);

    /// True while a trajectory is being tracked.
    bool isExecuting() const;

    /// Immediately stop trajectory tracking (resets to IDLE).
    void stop();

    // -----------------------------------------------------------------------
    // State query
    // -----------------------------------------------------------------------
    State getState() const { return state_; }

private:
    // -----------------------------------------------------------------------
    // GUI widgets
    // -----------------------------------------------------------------------

    // -- Planner parameters --
    flair::gui::ComboBox*    planner_mode_;      ///< Waypoint / Corridor
    flair::gui::DoubleSpinBox* safety_margin_;   ///< m, obstacle inflation radius
    flair::gui::DoubleSpinBox* max_velocity_;    ///< m/s
    flair::gui::DoubleSpinBox* max_acceleration_;///< m/s^2
    flair::gui::DoubleSpinBox* grid_resolution_; ///< m, occupancy grid voxel size
    flair::gui::DoubleSpinBox* replan_period_;   ///< s, receding-horizon period

    // -- Waypoints (up to 5, each with x/y/z spinboxes) --
    static const int kMaxWaypoints = 5;
    flair::gui::DoubleSpinBox* wp_x_[kMaxWaypoints];
    flair::gui::DoubleSpinBox* wp_y_[kMaxWaypoints];
    flair::gui::DoubleSpinBox* wp_z_[kMaxWaypoints];

    flair::gui::DoubleSpinBox* num_waypoints_;   ///< 1..5
    flair::gui::DoubleSpinBox* num_obstacles_;   ///< 0..5

    // -- Control buttons --
    flair::gui::PushButton* btn_plan_;
    flair::gui::PushButton* btn_execute_;
    flair::gui::PushButton* btn_stop_;

    // -- Status label --
    flair::gui::Label* status_label_;

    // -----------------------------------------------------------------------
    // Obstacles
    // -----------------------------------------------------------------------
    static const int kMaxObstacles = 5;
    ObstacleState obstacles_[kMaxObstacles];
    int num_tracked_obstacles_;       ///< How many obstacles are actually registered

    // -----------------------------------------------------------------------
    // Trajectory storage (double-buffer, synchronous — no threads needed)
    // -----------------------------------------------------------------------
    uav_planning::Trajectory<7> active_trajectory_;   ///< Currently tracked
    uav_planning::Trajectory<7> pending_trajectory_;  ///< Being prepared

    bool trajectory_valid_;     ///< active_trajectory_ contains a valid trajectory

    // -----------------------------------------------------------------------
    // Execution state
    // -----------------------------------------------------------------------
    State  state_;
    double execution_start_time_;  ///< t_actual when Execute was clicked
    double last_replan_time_;      ///< t_actual of last (re)plan

    // Cached UAV position at plan time (for occupancy grid construction)
    Eigen::Vector3d plan_start_pos_;

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /// Build occupancy grid and run the planner. @p uav_pos is current UAV
    /// world position. @p t_actual is current time in seconds.
    /// @return true on success.
    bool plan(const Eigen::Vector3d& uav_pos, double t_actual);

    /// Update all tracked obstacle positions/velocities from VRPN.
    void updateObstacles(double t_actual);

    /// Predict obstacle world position @p dt seconds into the future.
    Eigen::Vector3d predictObstaclePos(int idx, double dt) const;

    /// Set the status label text.
    void setStatus(const std::string& text);

    /// Allocate segment times proportional to Euclidean distance.
    /// Returns a vector of durations (one per segment, N-1 for N waypoints).
    std::vector<double> allocateTimes(
        const std::vector<Eigen::Vector3d>& wps,
        double max_vel) const;

    /// Build a minimum-snap trajectory from waypoints using the occupancy grid.
    /// Uses simple direct polynomial approach (no corridor):
    ///   - allocate times proportional to distance / max_vel
    ///   - solve minimum-snap QP per axis
    /// Stores result in @p traj. Returns true on success.
    bool solveMinSnap(const std::vector<Eigen::Vector3d>& waypoints,
                      const std::vector<double>& times,
                      uav_planning::Trajectory<7>& traj);

    /// Solve 1-D minimum-snap QP for a single axis.
    /// @param pos   Waypoint positions for this axis (N values, N-1 segments)
    /// @param times Segment durations (N-1 values)
    /// @param[out] traj  Output trajectory pieces for this axis (coefficients row 0)
    /// @return true on success
    bool solveMinSnap1D(const std::vector<double>& pos,
                        const std::vector<double>& times,
                        int axis,
                        uav_planning::Trajectory<7>& traj);

    // Non-copyable
    TrajectoryManager(const TrajectoryManager&);
    TrajectoryManager& operator=(const TrajectoryManager&);
};

#endif // TRAJECTORY_MANAGER_H
