// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file TrajectoryManager.h
 * \brief IODevice-based trajectory planner with min-snap optimization
 * \author Sergio Urzua, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2024
 * \version 2.0
 */

#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include <IODevice.h>
#include <Vector3D.h>
#include <Eigen/Core>
#include <vector>
#include <string>

namespace flair {
    namespace core {
        class Matrix;
        class io_data;
        // Time is a typedef (unsigned long long), not a class — no forward decl needed
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
        class SpinBox;
        class PushButton;
        class ComboBox;
        class GroupBox;
        class Label;
        class TabWidget;
        class Tab;
        class DataPlot1D;
        class DataPlot2D;
    }
}

/*!
 * \class TrajectoryManager
 * \brief Trajectory planner IODevice following Flair patterns
 *
 * Inherits from IODevice like TrajectoryGenerator2DCircle.
 * Provides min-snap trajectory generation with obstacle avoidance.
 * Outputs a 13-row Matrix: des_x/y/z, des_vx/vy/vz, des_ax/ay/az,
 * des_jx/jy/jz, and trajectory progress (0..1).
 *
 * Usage from ctrl1:
 *   traj_manager_ = new TrajectoryManager(positiongTab->NewRow(), "Trajectory Planner");
 *   // In control loop:
 *   traj_manager_->Update(GetTime());
 *   traj_manager_->GetPosition(xid);
 *   traj_manager_->GetSpeed(xidp);
 */
class TrajectoryManager : public flair::core::IODevice {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * \brief Constructor following Flair IODevice pattern
     * \param position Layout position for GUI widgets
     * \param name Object name
     */
    /// @param position   LayoutPosition for the settings widgets
    /// @param plots_tab   TabWidget where the trajectory plots tab will be created
    /// @param name        object name
    TrajectoryManager(const flair::gui::LayoutPosition *position,
                      flair::gui::TabWidget *plots_tab,
                      std::string name);

    /*!
     * \brief Destructor
     */
    ~TrajectoryManager();

    // -------------------------------------------------------
    // Trajectory evaluation (like TrajectoryGenerator2DCircle)
    // -------------------------------------------------------

    /*!
     * \brief Evaluate trajectory at given time, update output matrix
     * \param time Current Flair time in nanoseconds
     *
     * Evaluates the active trajectory, writes to the output matrix using
     * GetMutex/SetValueNoMutex/ReleaseMutex, then calls ProcessUpdate.
     */
    void Update(flair::core::Time time);

    /*!
     * \brief Get current desired position
     * \param pos Output position vector
     */
    void GetPosition(flair::core::Vector3Df &pos) const;

    /*!
     * \brief Get current desired velocity
     * \param vel Output velocity vector
     */
    void GetSpeed(flair::core::Vector3Df &vel) const;

    /*!
     * \brief Get current desired acceleration
     * \param acc Output acceleration vector
     */
    void GetAcceleration(flair::core::Vector3Df &acc) const;

    /*!
     * \brief Get current desired jerk
     * \param jerk Output jerk vector
     */
    void GetJerk(flair::core::Vector3Df &jerk) const;

    /*!
     * \brief Get output matrix (for external DataPlot wiring)
     * \return Pointer to the 13x1 output matrix
     */
    flair::core::Matrix *GetMatrix() const;

    // -------------------------------------------------------
    // Planning interface
    // -------------------------------------------------------

    /*!
     * \brief Plan a trajectory from current position to waypoints
     * \param current_pos Current UAV position
     * \param current_vel Current UAV velocity
     * \return true if planning succeeded
     */
    bool Plan(const flair::core::Vector3Df &current_pos,
              const flair::core::Vector3Df &current_vel);

    /*!
     * \brief Replan trajectory (receding horizon)
     * \param current_pos Current UAV position
     * \param current_vel Current UAV velocity
     * \return true if replanning succeeded
     */
    bool Replan(const flair::core::Vector3Df &current_pos,
                const flair::core::Vector3Df &current_vel);

    // -------------------------------------------------------
    // Lifecycle (like TrajectoryGenerator2DCircle)
    // -------------------------------------------------------

    /*!
     * \brief Start trajectory execution
     */
    void StartTraj();

    /*!
     * \brief Stop trajectory immediately
     */
    void StopTraj();

    /*!
     * \brief Check if trajectory is currently executing
     */
    bool IsRunning() const;

    /*!
     * \brief Get trajectory progress (0.0 to 1.0)
     */
    float GetProgress() const;

    // -------------------------------------------------------
    // Obstacle management
    // -------------------------------------------------------

    /*!
     * \brief Add a spherical obstacle
     * \param pos Obstacle center position
     * \param radius Obstacle radius
     */
    void AddObstacle(const flair::core::Vector3Df &pos, float radius);

    /*!
     * \brief Clear all obstacles
     */
    void ClearObstacles();

    /*!
     * \brief Update obstacle position
     * \param idx Obstacle index
     * \param pos New position
     */
    void UpdateObstaclePosition(int idx, const flair::core::Vector3Df &pos);

    /*!
     * \brief Update obstacle velocity (for prediction)
     * \param idx Obstacle index
     * \param vel Obstacle velocity
     */
    void UpdateObstacleVelocity(int idx, const flair::core::Vector3Df &vel);

private:
    /*!
     * \brief UpdateFrom override - empty (same as TrajectoryGenerator2DCircle)
     *
     * TrajectoryManager does not receive upstream data through the IODevice
     * chain. Update() is called explicitly from the control loop.
     */
    void UpdateFrom(const flair::core::io_data *data) {}

    // State machine
    enum class State : uint8_t {
        IDLE,
        PLANNING,
        PLANNED,
        EXECUTING,
        REPLANNING,
        HOLDING     ///< Trajectory finished, holding final position
    };

    State state_;

    // Output matrix (13x1): pos(3), vel(3), acc(3), jerk(3), progress(1)
    flair::core::Matrix *output_matrix_;

    // Cached trajectory state for GetPosition/GetSpeed/etc accessors
    flair::core::Vector3Df last_pos_;
    flair::core::Vector3Df last_vel_;
    flair::core::Vector3Df last_acc_;
    flair::core::Vector3Df last_jerk_;
    float progress_;

    // Trajectory storage (polynomial coefficients per segment)
    // Each segment: 3 x (ORDER+1) coefficient matrix, duration
    static const int POLY_ORDER = 7;  // min-snap = degree 7
    static const int MAX_SEGMENTS = 20;
    static const int MAX_GUI_WAYPOINTS = 7;  // GUI waypoints
    static const int MAX_WAYPOINTS = MAX_SEGMENTS + 1;
    static const int COEFFS_PER_SEG = POLY_ORDER + 1;  // 8

    struct TrajectorySegment {
        Eigen::Matrix<double, 3, 8> coeffs;  // 3 x (ORDER+1)
        double duration;
        TrajectorySegment() : coeffs(Eigen::Matrix<double, 3, 8>::Zero()), duration(0.0) {}
    };

    int num_segments_;
    TrajectorySegment segments_[MAX_SEGMENTS];
    double total_duration_;
    bool trajectory_valid_;

    // Timing
    double execution_start_time_;  // in seconds (from Flair Time)
    double last_replan_time_;

    // Waypoints (set via GUI or programmatically)
    int num_waypoints_;
    Eigen::Vector3d waypoints_[MAX_WAYPOINTS];
    Eigen::Vector3d start_vel_;
    Eigen::Vector3d end_vel_;

    // Obstacle list
    struct Obstacle {
        Eigen::Vector3d pos;
        Eigen::Vector3d vel;
        double radius;
    };
    static const int MAX_OBSTACLES = 16;
    int num_obstacles_;
    Obstacle obstacles_[MAX_OBSTACLES];

    // GUI widgets
    flair::gui::GroupBox *settings_box_;
    flair::gui::DoubleSpinBox *max_vel_;
    flair::gui::DoubleSpinBox *max_acc_;
    flair::gui::DoubleSpinBox *safety_margin_;
    flair::gui::DoubleSpinBox *replan_period_;
    flair::gui::DoubleSpinBox *grid_res_spin_;
    flair::gui::DoubleSpinBox *obstacle_radius_spin_;
    flair::gui::ComboBox *obstacle_avoidance_mode_;
    flair::gui::DoubleSpinBox *ws_xy_range_;
    flair::gui::DoubleSpinBox *ws_z_max_alt_;
    flair::gui::SpinBox *num_wp_spin_;
    flair::gui::DoubleSpinBox *wp_x_[MAX_GUI_WAYPOINTS];
    flair::gui::DoubleSpinBox *wp_y_[MAX_GUI_WAYPOINTS];
    flair::gui::DoubleSpinBox *wp_z_[MAX_GUI_WAYPOINTS];
    flair::gui::PushButton *plan_button_;
    flair::gui::PushButton *execute_button_;
    flair::gui::PushButton *stop_button_;
    flair::gui::Label *status_label_;

    // Internal helpers
    void ReadWaypointsFromGUI();
    bool SolveMinSnap();
    int LocateSegment(double t, double &t_local) const;
    Eigen::Vector3d EvalPos(double t) const;
    Eigen::Vector3d EvalVel(double t) const;
    Eigen::Vector3d EvalAcc(double t) const;
    Eigen::Vector3d EvalJer(double t) const;

    // -------------------------------------------------------
    // Obstacle avoidance pipeline (inline grid/JPS/SFC)
    // -------------------------------------------------------

    // Workspace bounds (NED: z negative = up, ground at z=0)
    double WS_X_MIN;
    double WS_X_MAX;
    double WS_Y_MIN;
    double WS_Y_MAX;
    double WS_Z_MIN;  // ceiling (most negative z = highest altitude)
    double WS_Z_MAX;  // ground at z=0

    // 3D occupancy grid (flat uint8_t array, row-major: ix * ny*nz + iy*nz + iz)
    std::vector<uint8_t> grid_data_;
    int grid_nx_, grid_ny_, grid_nz_;
    double grid_res_;
    Eigen::Vector3d grid_origin_;

    void InitGrid(double res);
    void ClearGrid();
    void MarkSphereOccupied(const Eigen::Vector3d &center, double radius);
    void MarkCylinderOccupied(double cx, double cy, double radius);
    bool IsOccupied(int ix, int iy, int iz) const;
    bool IsOccupiedWorld(const Eigen::Vector3d &pos) const;
    Eigen::Vector3d FindNearestFreeCell(const Eigen::Vector3d &pos) const;
    bool GridInBounds(int ix, int iy, int iz) const;
    Eigen::Vector3i WorldToGrid(const Eigen::Vector3d &pos) const;
    Eigen::Vector3d GridToWorld(int ix, int iy, int iz) const;
    bool IsSegmentFree(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const;

    // Build occupancy grid from current obstacles + ground plane
    void BuildOccupancyGrid();

    // A* path search on occupancy grid (26-connectivity)
    bool FindPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                  std::vector<Eigen::Vector3d> &path);
    std::vector<Eigen::Vector3d> SimplifyPath(const std::vector<Eigen::Vector3d> &input) const;

    // Safe Flight Corridor (AABB per segment)
    struct Corridor {
        Eigen::Vector3d lo, hi;  // AABB bounds
        bool contains(const Eigen::Vector3d &p) const {
            return p.x() >= lo.x() && p.x() <= hi.x() &&
                   p.y() >= lo.y() && p.y() <= hi.y() &&
                   p.z() >= lo.z() && p.z() <= hi.z();
        }
    };
    bool BuildCorridors(const std::vector<Eigen::Vector3d> &path,
                        double margin,
                        std::vector<Corridor> &corridors);

    // Corridor-constrained min-snap (iterative project-and-insert)
    bool SolveMinSnapConstrained(const std::vector<Eigen::Vector3d> &waypoints,
                                  const std::vector<Corridor> &corridors);

    // Full pipeline: grid → JPS → SFC → constrained min-snap
    bool PlanWithObstacleAvoidance();
};

#endif // TRAJECTORY_MANAGER_H
