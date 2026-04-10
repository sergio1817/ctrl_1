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
#include <Eigen/Dense>
#include <vector>
#include <string>

// GCOPTER headers
#include "trajectory.hpp"
#include "gcopter.hpp"
#include "firi.hpp"

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
 * Provides min-snap / GCOPTER trajectory generation with obstacle avoidance.
 * Outputs a 13-row Matrix: des_x/y/z, des_vx/vy/vz, des_ax/ay/az,
 * des_jx/jy/jz, progress.
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
    void Update(flair::core::Time time,
                const flair::core::Vector3Df &uav_pos = flair::core::Vector3Df(0,0,0),
                const flair::core::Vector3Df &uav_vel = flair::core::Vector3Df(0,0,0));
    void GetPosition(flair::core::Vector3Df &pos) const;
    void GetSpeed(flair::core::Vector3Df &vel) const;
    void GetAcceleration(flair::core::Vector3Df &acc) const;
    void GetJerk(flair::core::Vector3Df &jerk) const;
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

    // Output matrix (13x1): pos(3), vel(3), acc(3), jerk(3), progress(1)
    flair::core::Matrix *output_matrix_;

    // Cached trajectory state
    flair::core::Vector3Df last_pos_;
    flair::core::Vector3Df last_vel_;
    flair::core::Vector3Df last_acc_;
    flair::core::Vector3Df last_jerk_;
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

    // GCOPTER/MINCO backend
    bool use_gcopter_;  // true when GCOPTER/MINCO backend is selected
    Trajectory<5> gcopter_traj_;  // stored GCOPTER trajectory (degree 5)
    bool use_gcopter_traj_;        // true when GCOPTER produced the trajectory
    bool gcopter_traj_valid_;     // true when gcopter_traj_ has valid data

    // --- SOTA Upgrade 2: Gradient-based time allocation ---
    bool time_opt_enabled_;
    double kT_;                  // time penalty weight

    // --- SOTA Upgrade 3: TOPP-RA post-processing (Pham & Pham 2018) ---
    bool topp_ra_enabled_;

    // TOPP-RA pre-allocated arrays (50 grid points per segment, max 20 segments = 1000+1)
    static const int TOPPRA_SAMPLES_PER_SEG = 50;
    static const int TOPPRA_MAX_GRID = MAX_SEGMENTS * TOPPRA_SAMPLES_PER_SEG + 1;
    double toppra_s_[TOPPRA_MAX_GRID];           // path parameter grid
    double toppra_ds_[TOPPRA_MAX_GRID];          // delta-s per interval
    double toppra_x_max_vel_[TOPPRA_MAX_GRID];   // velocity-limited x = sdot^2
    double toppra_K_lo_[TOPPRA_MAX_GRID];        // controllable set lower bound
    double toppra_K_hi_[TOPPRA_MAX_GRID];        // controllable set upper bound
    double toppra_x_[TOPPRA_MAX_GRID];           // forward pass optimal x = sdot^2
    double toppra_u_[TOPPRA_MAX_GRID];           // forward pass optimal u = sddot
    double toppra_p_prime_[TOPPRA_MAX_GRID][3];  // p'(s) tangent
    double toppra_p_dprime_[TOPPRA_MAX_GRID][3]; // p''(s) curvature

    // --- SOTA Upgrade 5: Receding-horizon replanning ---
    double replan_horizon_;
    double blend_duration_;
    Trajectory<5> contingency_traj_;     // decelerate-to-hover backup
    bool contingency_valid_;
    Trajectory<5> prev_traj_;            // previous trajectory for blending
    bool prev_traj_valid_;
    double prev_traj_start_time_;

    // Current UAV state (updated each tick from ctrl1)
    Eigen::Vector3d current_uav_pos_;
    Eigen::Vector3d current_uav_vel_;
    Eigen::Vector3d current_uav_acc_;

    // Timing
    double execution_start_time_;
    bool   execution_time_set_;   ///< true once execution_start_time_ set from a real tick
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

    // Planner backend selection
    flair::gui::ComboBox *planner_backend_;

    // Internal helpers — original
    void ReadWaypointsFromGUI();
    bool SolveMinSnap();
    int LocateSegment(double t, double &t_local) const;
    Eigen::Vector3d EvalPos(double t) const;
    Eigen::Vector3d EvalVel(double t) const;
    Eigen::Vector3d EvalAcc(double t) const;
    Eigen::Vector3d EvalJer(double t) const;

    // GCOPTER/MINCO backend
    bool SolveGCOPTER();
    std::vector<Eigen::Vector3d> GetNearbyObstaclePoints(
        const Eigen::Vector3d &seg_start,
        const Eigen::Vector3d &seg_end,
        double radius) const;

    // --- SOTA Upgrade 2: Gradient-based time allocation (Richter et al. 2016) ---
    bool OptimizeTimeAllocation(Eigen::VectorXd &ts,
                                const Eigen::Matrix3Xd &inPs,
                                const Eigen::Matrix3d &headPVA,
                                const Eigen::Matrix3d &tailPVA);

    // --- SOTA Upgrade 3: TOPP-RA post-processing ---
    bool ApplyTOPPRA();

    // --- Corridor struct (needed by GenerateCorridors and BuildCorridors) ---
    struct Corridor {
        Eigen::Vector3d lo, hi;
        bool contains(const Eigen::Vector3d &p) const {
            return p.x() >= lo.x() && p.x() <= hi.x() &&
                   p.y() >= lo.y() && p.y() <= hi.y() &&
                   p.z() >= lo.z() && p.z() <= hi.z();
        }
    };

    // --- SOTA Upgrade 4: FIRI corridor generation ---
    bool GenerateCorridors(const std::vector<Eigen::Vector3d> &waypoints,
                           std::vector<Eigen::MatrixX4d> &hPolytopes,
                           std::vector<Corridor> &aabb_corridors);

    // --- SOTA Upgrade 5: Receding-horizon replanning ---
    void GenerateContingencyTrajectory(const Eigen::Vector3d &pos,
                                       const Eigen::Vector3d &vel,
                                       const Eigen::Vector3d &acc);

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

    // --- SOTA: Incremental occupancy grid ---
    bool grid_dirty_;
    bool grid_initialized_;
    Eigen::Vector3d prev_obstacle_pos_[MAX_OBSTACLES];
    int prev_num_obstacles_;
    struct InflationOffset { int dx; int dy; };
    std::vector<InflationOffset> inflation_template_;
    double inflation_template_radius_;

    // --- SOTA: Pipeline caching ---
    bool cache_valid_;

    // --- SOTA: MINCO warm start ---
    bool warm_start_enabled_;
    Eigen::VectorXd prev_trajectory_times_;
    bool prev_coeffs_valid_;

    // --- SOTA: Distance field ---
    bool distance_field_valid_;

    // --- ESDF-Lite distance field ---
    std::vector<float> distance_field_;
    void ComputeDistanceField();
    double GetObstacleDistance(const Eigen::Vector3d &world_pos) const;

    // --- Real-time safety monitor ---
    bool safety_monitor_enabled_;
    double emergency_distance_;
    double replan_distance_;
    double last_safety_replan_time_;

    // --- Adaptive prediction horizon ---
    double last_update_time_;

    std::vector<float> astar_gcost_;
    std::vector<int>   astar_parent_;
    std::vector<int8_t> jps_dir_x_;   ///< JPS arrival direction per cell
    std::vector<int8_t> jps_dir_y_;
    std::vector<int8_t> jps_dir_z_;

    // --- SOTA: Bucket queue for JPS/A* (O(1) push/pop) ---
    struct BucketQueue {
        std::vector<std::vector<int>> buckets;
        int min_bucket;
        int num_buckets;
        double inv_resolution;

        void init(int max_buckets, double bucket_res) {
            num_buckets = max_buckets;
            inv_resolution = 1.0 / bucket_res;
            buckets.resize(max_buckets);
            min_bucket = 0;
        }
        void clear() {
            for (int i = 0; i < num_buckets; ++i) buckets[i].clear();
            min_bucket = 0;
        }
        void push(int node, double cost) {
            int b = static_cast<int>(cost * inv_resolution);
            if (b < 0) b = 0;
            if (b >= num_buckets) b = num_buckets - 1;
            buckets[b].push_back(node);
            if (b < min_bucket) min_bucket = b;
        }
        bool empty() const {
            for (int b = min_bucket; b < num_buckets; ++b)
                if (!buckets[b].empty()) return false;
            return true;
        }
        int pop() {
            while (min_bucket < num_buckets && buckets[min_bucket].empty())
                ++min_bucket;
            if (min_bucket >= num_buckets) return -1;
            int node = buckets[min_bucket].back();
            buckets[min_bucket].pop_back();
            return node;
        }
    };
    BucketQueue bucket_queue_;

    // -------------------------------------------------------
    // JPS 3D neighbor pruning tables
    // -------------------------------------------------------
    struct JPS3DNeib {
        // ns[id][axis][dev] — natural successor directions per (dx,dy,dz)
        int ns[27][3][26];
        // f1[id][axis][dev] — forced neighbor obstacle-check positions
        int f1[27][3][12];
        // f2[id][axis][dev] — forced neighbor add directions
        int f2[27][3][12];
        // nsz[norm1][0] = num natural, nsz[norm1][1] = num forced
        static const int nsz[4][2];

        JPS3DNeib();
    private:
        void Neib(int dx, int dy, int dz, int norm1, int dev,
                  int& tx, int& ty, int& tz);
        void FNeib(int dx, int dy, int dz, int norm1, int dev,
                   int& fx, int& fy, int& fz,
                   int& nx, int& ny, int& nz);
    };
    JPS3DNeib jps_neib_;  // constructed once in InitGrid

    int jps_goal_x_, jps_goal_y_, jps_goal_z_;  // goal grid coords for JumpJPS

    bool JumpJPS(int x, int y, int z, int dx, int dy, int dz,
                 int& jx, int& jy, int& jz);
    bool HasForcedJPS(int x, int y, int z, int dx, int dy, int dz);
    bool FindPathJPS(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                     std::vector<Eigen::Vector3d> &path);
    bool FindPathAStar(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                       std::vector<Eigen::Vector3d> &path);

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
    void BuildOccupancyGridIncremental();
    void PrecomputeInflationTemplate(double radius);
    bool ObstaclesMoved() const;

    bool FindPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                  std::vector<Eigen::Vector3d> &path);
    std::vector<Eigen::Vector3d> SimplifyPath(const std::vector<Eigen::Vector3d> &input) const;

    bool BuildCorridors(const std::vector<Eigen::Vector3d> &path,
                        double margin,
                        std::vector<Corridor> &corridors);

    bool SolveMinSnapConstrained(const std::vector<Eigen::Vector3d> &waypoints,
                                  const std::vector<Corridor> &corridors);

    bool PlanWithObstacleAvoidance();
};

#endif // TRAJECTORY_MANAGER_H
