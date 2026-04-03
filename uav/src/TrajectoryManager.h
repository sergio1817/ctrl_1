// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file TrajectoryManager.h
 * \brief IODevice-based trajectory planner with min-snap optimization
 * \author Sergio Urzua, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2024
 * \version 3.0
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
 * Provides min-snap / AM-Traj trajectory generation with obstacle avoidance.
 * Outputs a 15-row Matrix: des_x/y/z, des_vx/vy/vz, des_ax/ay/az,
 * des_jx/jy/jz, progress, des_yaw, des_yaw_rate.
 */
class TrajectoryManager : public flair::core::IODevice {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TrajectoryManager(const flair::gui::LayoutPosition *position,
                      flair::gui::TabWidget *plots_tab,
                      std::string name);
    ~TrajectoryManager();

    // -------------------------------------------------------
    // Trajectory evaluation
    // -------------------------------------------------------
    void Update(flair::core::Time time);
    void GetPosition(flair::core::Vector3Df &pos) const;
    void GetSpeed(flair::core::Vector3Df &vel) const;
    void GetAcceleration(flair::core::Vector3Df &acc) const;
    void GetJerk(flair::core::Vector3Df &jerk) const;
    float GetDesiredYaw() const;
    float GetDesiredYawRate() const;
    flair::core::Matrix *GetMatrix() const;

    // -------------------------------------------------------
    // Planning interface
    // -------------------------------------------------------
    bool Plan(const flair::core::Vector3Df &current_pos,
              const flair::core::Vector3Df &current_vel);
    bool Replan(const flair::core::Vector3Df &current_pos,
                const flair::core::Vector3Df &current_vel);

    // -------------------------------------------------------
    // Lifecycle
    // -------------------------------------------------------
    void StartTraj();
    void StopTraj();
    bool IsRunning() const;
    float GetProgress() const;

    // -------------------------------------------------------
    // Obstacle management
    // -------------------------------------------------------
    void AddObstacle(const flair::core::Vector3Df &pos, float radius);
    void ClearObstacles();
    void UpdateObstaclePosition(int idx, const flair::core::Vector3Df &pos);
    void UpdateObstacleVelocity(int idx, const flair::core::Vector3Df &vel);

    // Phase 6: Camera-based obstacle feed
    void AddCameraObstacle(const flair::core::Vector3Df &pos, float radius);

private:
    void UpdateFrom(const flair::core::io_data *data) {}

    // State machine
    enum class State : uint8_t {
        IDLE,
        PLANNING,
        PLANNED,
        EXECUTING,
        REPLANNING,
        HOLDING
    };

    State state_;

    // Output matrix (15x1): pos(3), vel(3), acc(3), jerk(3), progress(1), yaw(1), yaw_rate(1)
    flair::core::Matrix *output_matrix_;

    // Cached trajectory state
    flair::core::Vector3Df last_pos_;
    flair::core::Vector3Df last_vel_;
    flair::core::Vector3Df last_acc_;
    flair::core::Vector3Df last_jerk_;
    float last_yaw_;
    float last_yaw_rate_;
    float progress_;

    // Trajectory storage (polynomial coefficients per segment)
    static const int POLY_ORDER = 7;  // min-snap = degree 7
    static const int MAX_SEGMENTS = 20;
    static const int MAX_GUI_WAYPOINTS = 7;
    static const int MAX_WAYPOINTS = MAX_SEGMENTS + 1;
    static const int COEFFS_PER_SEG = POLY_ORDER + 1;  // 8

    struct TrajectorySegment {
        Eigen::Matrix<double, 3, 8> coeffs;  // 3 x (ORDER+1) for min-snap
        double duration;
        TrajectorySegment() : coeffs(Eigen::Matrix<double, 3, 8>::Zero()), duration(0.0) {}
    };

    int num_segments_;
    TrajectorySegment segments_[MAX_SEGMENTS];
    double total_duration_;
    bool trajectory_valid_;

    // Phase 1: AM-Traj backend flag
    bool use_amtraj_;  // true when AM-Traj backend is selected

    // Phase 2: Yaw trajectory (degree-3 polynomial per segment)
    double yaw_coeffs_[MAX_SEGMENTS][4];  // a0 + a1*t + a2*t^2 + a3*t^3

    // Timing
    double execution_start_time_;
    double last_replan_time_;

    // Waypoints
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

    // GUI widgets — original
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

    // Phase 1: AM-Traj GUI widgets
    flair::gui::ComboBox *planner_backend_;
    flair::gui::DoubleSpinBox *amtraj_wt_;
    flair::gui::SpinBox *amtraj_max_iter_;

    // Phase 2: Yaw GUI widgets
    flair::gui::ComboBox *yaw_mode_;
    flair::gui::DoubleSpinBox *fixed_yaw_;

    // Phase 4: TOPP-RA GUI widgets
    flair::gui::ComboBox *postproc_mode_;

    // Phase 5: Spatio-temporal corridor GUI widgets
    flair::gui::DoubleSpinBox *prediction_horizon_;
    flair::gui::DoubleSpinBox *uncertainty_growth_;

    // Internal helpers — original
    void ReadWaypointsFromGUI();
    bool SolveMinSnap();
    int LocateSegment(double t, double &t_local) const;
    Eigen::Vector3d EvalPos(double t) const;
    Eigen::Vector3d EvalVel(double t) const;
    Eigen::Vector3d EvalAcc(double t) const;
    Eigen::Vector3d EvalJer(double t) const;

    // Phase 1: AM-Traj backend
    bool SolveAmTraj();

    // Phase 2: Yaw trajectory helpers
    void ComputeYawTrajectory();
    double EvalYaw(double t) const;
    double EvalYawRate(double t) const;

    // Phase 4: TOPP-RA post-processing
    void ReparametrizeTopp();

    // -------------------------------------------------------
    // Obstacle avoidance pipeline
    // -------------------------------------------------------

    double WS_X_MIN;
    double WS_X_MAX;
    double WS_Y_MIN;
    double WS_Y_MAX;
    double WS_Z_MIN;
    double WS_Z_MAX;

    std::vector<uint8_t> grid_data_;
    int grid_nx_, grid_ny_, grid_nz_;
    double grid_res_;
    Eigen::Vector3d grid_origin_;

    std::vector<float> astar_gcost_;
    std::vector<int>   astar_parent_;

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

    void BuildOccupancyGrid();

    bool FindPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                  std::vector<Eigen::Vector3d> &path);
    std::vector<Eigen::Vector3d> SimplifyPath(const std::vector<Eigen::Vector3d> &input) const;

    struct Corridor {
        Eigen::Vector3d lo, hi;
        bool contains(const Eigen::Vector3d &p) const {
            return p.x() >= lo.x() && p.x() <= hi.x() &&
                   p.y() >= lo.y() && p.y() <= hi.y() &&
                   p.z() >= lo.z() && p.z() <= hi.z();
        }
    };
    bool BuildCorridors(const std::vector<Eigen::Vector3d> &path,
                        double margin,
                        std::vector<Corridor> &corridors);

    bool SolveMinSnapConstrained(const std::vector<Eigen::Vector3d> &waypoints,
                                  const std::vector<Corridor> &corridors);

    bool PlanWithObstacleAvoidance();
};

#endif // TRAJECTORY_MANAGER_H
