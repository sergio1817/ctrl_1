/**
 * trajectory_planner.hpp
 *
 * Top-level UAV trajectory planner that integrates:
 *   1. OccupancyGrid3D    — obstacle representation
 *   2. JPS3D              — fast path search
 *   3. SafeFlightCorridor — convex decomposition of free space
 *   4. solveMinSnap       — minimum-snap polynomial trajectory optimisation
 *
 * Pipeline:
 *   plan(start, goal) → JPS path → SFC construction → time allocation
 *                     → min-snap solve → Trajectory<7>
 *
 * Thread safety:
 *   The trajectory can be read while a new one is being computed using a
 *   mutex-protected double-buffer swap. The reader always gets the last
 *   committed (complete) trajectory.
 *
 * Compatible with GCC 4.9 / C++11. Depends only on Eigen3.
 */

#ifndef UAV_TRAJECTORY_PLANNER_HPP
#define UAV_TRAJECTORY_PLANNER_HPP

#include <vector>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#include "trajectory.hpp"
#include "minsnap_solver.hpp"
#include "occupancy_grid.hpp"
#include "jps3d.hpp"
#include "convex_decomp.hpp"

namespace uav_planning {

// =============================================================================
// PlannerConfig
// =============================================================================

/**
 * @brief Configuration parameters for the TrajectoryPlanner.
 */
struct PlannerConfig
{
    double grid_resolution;   ///< Voxel size (metres), default 0.2 m
    double robot_radius;      ///< Robot radius for SFC clearance (metres), default 0.3 m
    double safety_margin;     ///< Additional clearance beyond robot_radius (metres), default 0.1 m
    double max_vel;           ///< Maximum UAV speed (m/s), default 5.0
    double max_acc;           ///< Maximum UAV acceleration (m/s^2), default 3.0
    double workspace_x_min;   ///< Workspace extents
    double workspace_x_max;
    double workspace_y_min;
    double workspace_y_max;
    double workspace_z_min;
    double workspace_z_max;

    PlannerConfig()
        : grid_resolution(0.2)
        , robot_radius(0.3)
        , safety_margin(0.1)
        , max_vel(5.0)
        , max_acc(3.0)
        , workspace_x_min(-50.0)
        , workspace_x_max( 50.0)
        , workspace_y_min(-50.0)
        , workspace_y_max( 50.0)
        , workspace_z_min(  0.0)
        , workspace_z_max( 20.0)
    {}
};

// =============================================================================
// PlannerResult
// =============================================================================

/**
 * @brief Result of a planning query.
 */
enum PlannerStatus
{
    PLANNER_SUCCESS = 0,
    PLANNER_FAILED_PATH_NOT_FOUND,
    PLANNER_FAILED_CORRIDOR_BUILD,
    PLANNER_FAILED_TRAJECTORY_SOLVE,
    PLANNER_FAILED_INVALID_INPUT
};

// =============================================================================
// TrajectoryPlanner
// =============================================================================

/**
 * @brief High-level UAV trajectory planner.
 *
 * Typical usage:
 * @code
 *   PlannerConfig cfg;
 *   cfg.grid_resolution = 0.2;
 *   cfg.robot_radius    = 0.3;
 *
 *   TrajectoryPlanner planner(cfg);
 *   planner.addSphereObstacle(Eigen::Vector3d(5, 0, 2), 1.5);
 *
 *   Eigen::Vector3d start(0,0,1), goal(10,0,2);
 *   Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
 *   Eigen::Vector3d goal_vel  = Eigen::Vector3d::Zero();
 *
 *   if (planner.plan(start, start_vel, goal, goal_vel, 5.0, 3.0))
 *   {
 *       const Trajectory<7>& traj = planner.getTrajectory();
 *       Eigen::Vector3d pos = traj.getPos(1.0);
 *   }
 * @endcode
 */
class TrajectoryPlanner
{
public:
    /**
     * @brief Construct a planner with the given configuration.
     */
    explicit TrajectoryPlanner(const PlannerConfig& config)
        : config_(config)
        , last_status_(PLANNER_FAILED_PATH_NOT_FOUND)
    {
        resetGrid();
    }

    /**
     * @brief Construct a planner with default configuration.
     */
    TrajectoryPlanner()
        : last_status_(PLANNER_FAILED_PATH_NOT_FOUND)
    {
        resetGrid();
    }

    // -------------------------------------------------------------------------
    // Grid management
    // -------------------------------------------------------------------------

    /**
     * @brief Replace the occupancy grid with the given one.
     *
     * The planner takes a copy of the grid.
     */
    void setOccupancyGrid(const OccupancyGrid3D& grid)
    {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        grid_.reset(new OccupancyGrid3D(grid));
    }

    /**
     * @brief Add a spherical obstacle to the current grid.
     *
     * @param center  World-space centre of the sphere.
     * @param radius  Radius of the sphere (metres).
     */
    void addSphereObstacle(const Eigen::Vector3d& center, double radius)
    {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        ensureGrid();
        grid_->addSphereObstacle(center, radius);
    }

    /**
     * @brief Add an axis-aligned box obstacle.
     *
     * @param min_corner  Minimum corner of the box.
     * @param max_corner  Maximum corner of the box.
     */
    void addBoxObstacle(const Eigen::Vector3d& min_corner,
                        const Eigen::Vector3d& max_corner)
    {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        ensureGrid();
        grid_->addBoxObstacle(min_corner, max_corner);
    }

    /**
     * @brief Clear all obstacles.
     */
    void clearObstacles()
    {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        ensureGrid();
        grid_->clear();
    }

    /**
     * @brief Update the planner configuration.
     *
     * This resets the occupancy grid to match the new workspace bounds.
     */
    void setConfig(const PlannerConfig& config)
    {
        config_ = config;
        resetGrid();
    }

    const PlannerConfig& getConfig() const { return config_; }

    // -------------------------------------------------------------------------
    // Planning
    // -------------------------------------------------------------------------

    /**
     * @brief Plan a trajectory from @p start to @p goal.
     *
     * Executes the full pipeline:
     *   JPS3D path search → SFC construction → time allocation → min-snap solve.
     *
     * On success, the new trajectory is atomically committed (mutex swap) so
     * that concurrent readers always see a consistent state.
     *
     * @param start      Start position (world, metres).
     * @param start_vel  Start velocity (m/s).
     * @param goal       Goal position (world, metres).
     * @param goal_vel   Goal velocity (m/s).
     * @param max_vel    Maximum speed override (m/s). If <= 0, uses config value.
     * @param max_acc    Maximum acceleration override (m/s^2). If <= 0, uses config value.
     * @return           true on success.
     */
    bool plan(const Eigen::Vector3d& start,
              const Eigen::Vector3d& start_vel,
              const Eigen::Vector3d& goal,
              const Eigen::Vector3d& goal_vel,
              double max_vel = -1.0,
              double max_acc = -1.0)
    {
        if (max_vel <= 0.0) max_vel = config_.max_vel;
        if (max_acc <= 0.0) max_acc = config_.max_acc;

        // Validate inputs
        if ((goal - start).norm() < 1e-6)
        {
            last_status_ = PLANNER_FAILED_INVALID_INPUT;
            return false;
        }

        // ---- Step 1: path search ----
        OccupancyGrid3D grid_copy = getGridCopy();

        JPS3D path_planner(grid_copy);
        if (!path_planner.search(start, goal))
        {
            last_status_ = PLANNER_FAILED_PATH_NOT_FOUND;
            return false;
        }

        std::vector<Eigen::Vector3d> raw_path = path_planner.getPath();
        if (raw_path.size() < 2)
        {
            last_status_ = PLANNER_FAILED_PATH_NOT_FOUND;
            return false;
        }

        // ---- Step 2: SFC construction ----
        double clearance = config_.robot_radius + config_.safety_margin;
        SafeFlightCorridor sfc;
        if (!sfc.build(raw_path, grid_copy, clearance))
        {
            // SFC failed — try with tighter clearance (fallback)
            if (!sfc.build(raw_path, grid_copy, config_.robot_radius * 0.5))
            {
                last_status_ = PLANNER_FAILED_CORRIDOR_BUILD;
                return false;
            }
        }

        // ---- Step 3: Time allocation ----
        std::vector<double> durations;
        try
        {
            durations = allocateTimeTrapezoidal(raw_path, max_vel, max_acc);
        }
        catch (const std::exception&)
        {
            last_status_ = PLANNER_FAILED_INVALID_INPUT;
            return false;
        }

        // ---- Step 4: Minimum-snap optimisation ----
        BoundaryCondition start_bc(start_vel,
                                   Eigen::Vector3d::Zero(),
                                   Eigen::Vector3d::Zero());
        BoundaryCondition end_bc(goal_vel,
                                 Eigen::Vector3d::Zero(),
                                 Eigen::Vector3d::Zero());

        Trajectory<7> new_traj;
        try
        {
            new_traj = solveMinSnap(raw_path, durations, start_bc, end_bc);
        }
        catch (const std::exception&)
        {
            last_status_ = PLANNER_FAILED_TRAJECTORY_SOLVE;
            return false;
        }

        if (new_traj.empty())
        {
            last_status_ = PLANNER_FAILED_TRAJECTORY_SOLVE;
            return false;
        }

        // ---- Commit results (thread-safe swap) ----
        {
            std::lock_guard<std::mutex> lock(traj_mutex_);
            committed_traj_  = new_traj;
            committed_path_  = raw_path;
            committed_sfc_   = sfc.getPolyhedra();
        }

        last_status_ = PLANNER_SUCCESS;
        return true;
    }

    /**
     * @brief Replan from the current position and velocity towards a new goal.
     *
     * Intended for receding-horizon replanning: discards the old trajectory
     * and plans fresh from the current UAV state.
     *
     * @param current_pos  Current UAV world position.
     * @param current_vel  Current UAV velocity.
     * @param goal         New goal position.
     * @param max_vel      Maximum speed (m/s). If <= 0, uses config value.
     * @param max_acc      Maximum acceleration (m/s^2). If <= 0, uses config value.
     * @return             true on success.
     */
    bool replan(const Eigen::Vector3d& current_pos,
                const Eigen::Vector3d& current_vel,
                const Eigen::Vector3d& goal,
                double max_vel = -1.0,
                double max_acc = -1.0)
    {
        // Retrieve goal velocity from current committed trajectory if available,
        // otherwise use zero.
        Eigen::Vector3d goal_vel = Eigen::Vector3d::Zero();
        return plan(current_pos, current_vel, goal, goal_vel, max_vel, max_acc);
    }

    // -------------------------------------------------------------------------
    // Result accessors (thread-safe reads)
    // -------------------------------------------------------------------------

    /**
     * @brief Return a copy of the last committed trajectory.
     *
     * Thread-safe: takes a lock and copies the trajectory.
     */
    Trajectory<7> getTrajectory() const
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        return committed_traj_;
    }

    /**
     * @brief Return a copy of the last committed path (waypoints).
     */
    std::vector<Eigen::Vector3d> getPath() const
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        return committed_path_;
    }

    /**
     * @brief Return a copy of the last committed SFC polyhedra.
     */
    std::vector<ConvexPolyhedron> getCorridor() const
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        return committed_sfc_;
    }

    /**
     * @brief Return the status of the last plan() or replan() call.
     */
    PlannerStatus getLastStatus() const { return last_status_; }

    /**
     * @brief Return a human-readable description of the last status.
     */
    const char* getStatusString() const
    {
        switch (last_status_)
        {
            case PLANNER_SUCCESS:                    return "Success";
            case PLANNER_FAILED_PATH_NOT_FOUND:      return "Path not found";
            case PLANNER_FAILED_CORRIDOR_BUILD:      return "Corridor build failed";
            case PLANNER_FAILED_TRAJECTORY_SOLVE:    return "Trajectory solve failed";
            case PLANNER_FAILED_INVALID_INPUT:       return "Invalid input";
            default:                                 return "Unknown";
        }
    }

    // -------------------------------------------------------------------------
    // Utility
    // -------------------------------------------------------------------------

    /**
     * @brief Evaluate the current trajectory at a given time (thread-safe).
     *
     * @param t  Global time along the trajectory.
     * @return   Position, or zero vector if no trajectory is available.
     */
    Eigen::Vector3d evalPos(double t) const
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (committed_traj_.empty()) return Eigen::Vector3d::Zero();
        return committed_traj_.getPos(t);
    }

    Eigen::Vector3d evalVel(double t) const
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (committed_traj_.empty()) return Eigen::Vector3d::Zero();
        return committed_traj_.getVel(t);
    }

    Eigen::Vector3d evalAcc(double t) const
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (committed_traj_.empty()) return Eigen::Vector3d::Zero();
        return committed_traj_.getAcc(t);
    }

    /**
     * @brief Check whether a world-space position is safe
     *        (inside at least one SFC corridor polyhedron).
     *
     * @param pos  World position to check.
     * @return     true if pos is covered by the corridor.
     */
    bool isInCorridor(const Eigen::Vector3d& pos) const
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        for (size_t i = 0; i < committed_sfc_.size(); i++)
        {
            if (committed_sfc_[i].contains(pos)) return true;
        }
        return false;
    }

    /**
     * @brief Return true if a valid trajectory is available.
     */
    bool hasTrajectory() const
    {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        return !committed_traj_.empty();
    }

private:
    PlannerConfig config_;
    PlannerStatus last_status_;

    // Occupancy grid (protected by grid_mutex_)
    mutable std::mutex grid_mutex_;
    std::shared_ptr<OccupancyGrid3D> grid_;

    // Results (protected by traj_mutex_)
    mutable std::mutex traj_mutex_;
    Trajectory<7> committed_traj_;
    std::vector<Eigen::Vector3d> committed_path_;
    std::vector<ConvexPolyhedron> committed_sfc_;

    /**
     * @brief Re-create the occupancy grid to match current config workspace bounds.
     */
    void resetGrid()
    {
        std::lock_guard<std::mutex> lock(grid_mutex_);

        double res = config_.grid_resolution;
        double lx = config_.workspace_x_max - config_.workspace_x_min;
        double ly = config_.workspace_y_max - config_.workspace_y_min;
        double lz = config_.workspace_z_max - config_.workspace_z_min;

        int nx = std::max(1, static_cast<int>(std::ceil(lx / res)));
        int ny = std::max(1, static_cast<int>(std::ceil(ly / res)));
        int nz = std::max(1, static_cast<int>(std::ceil(lz / res)));

        Eigen::Vector3d origin(config_.workspace_x_min,
                               config_.workspace_y_min,
                               config_.workspace_z_min);

        grid_.reset(new OccupancyGrid3D(origin, nx, ny, nz, res));
    }

    /**
     * @brief Ensure the grid is allocated (should always be true after construction).
     */
    void ensureGrid()
    {
        if (!grid_)
            resetGrid();
    }

    /**
     * @brief Return a copy of the current occupancy grid (thread-safe).
     */
    OccupancyGrid3D getGridCopy() const
    {
        std::lock_guard<std::mutex> lock(grid_mutex_);
        if (!grid_)
        {
            // Return a default grid
            Eigen::Vector3d origin(config_.workspace_x_min,
                                   config_.workspace_y_min,
                                   config_.workspace_z_min);
            double res = config_.grid_resolution;
            int nx = std::max(1, static_cast<int>(std::ceil(
                (config_.workspace_x_max - config_.workspace_x_min) / res)));
            int ny = std::max(1, static_cast<int>(std::ceil(
                (config_.workspace_y_max - config_.workspace_y_min) / res)));
            int nz = std::max(1, static_cast<int>(std::ceil(
                (config_.workspace_z_max - config_.workspace_z_min) / res)));
            return OccupancyGrid3D(origin, nx, ny, nz, res);
        }
        return *grid_;
    }
};

} // namespace uav_planning

#endif // UAV_TRAJECTORY_PLANNER_HPP
