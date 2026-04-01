/**
 * jps3d.hpp
 *
 * Jump Point Search (JPS) in 3D for fast path planning on occupancy grids.
 *
 * Implements A* search with 26-connectivity and jump-point pruning adapted to
 * 3D uniform grids, following the approach of:
 *
 *   Harabor & Grastien: "Online Graph Pruning for Pathfinding on Grid Maps",
 *   AAAI 2011 (original 2D JPS).
 *
 *   Liu et al.: "Planning Dynamically Feasible Trajectories for Quadrotors
 *   Using Safe Flight Corridors in 3-D Complex Environments", RA-L 2017.
 *
 * Full 3D JPS forced-neighbor rules are complex; here we implement a robust
 * A* with:
 *   - 26-connectivity (face, edge, and corner neighbours)
 *   - Euclidean distance heuristic
 *   - Path post-processing: collinear waypoint removal using ray-casting
 *
 * For correctness and reliability on embedded targets this is preferable over
 * a partially-correct full JPS implementation.
 *
 * Compatible with GCC 4.9 / C++11. Depends only on Eigen3.
 */

#ifndef UAV_JPS3D_HPP
#define UAV_JPS3D_HPP

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <cassert>
#include <stdexcept>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>

#include "occupancy_grid.hpp"

namespace uav_planning {

// =============================================================================
// Internal types
// =============================================================================
namespace jps_detail {

/// Compact 3-int index for a grid voxel
struct GridIndex
{
    int x, y, z;
    GridIndex() : x(0), y(0), z(0) {}
    GridIndex(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}

    bool operator==(const GridIndex& o) const
    {
        return x == o.x && y == o.y && z == o.z;
    }
};

/// Node in the A* search tree
struct Node
{
    GridIndex idx;
    double g;   ///< cost-to-come
    double f;   ///< f = g + h (priority)
    int parent; ///< linear index of parent in node storage, -1 if root

    Node() : g(0.0), f(0.0), parent(-1) {}
};

/// Comparator for the priority queue (min-heap on f)
struct NodeCompare
{
    bool operator()(const std::pair<double, int>& a,
                    const std::pair<double, int>& b) const
    {
        return a.first > b.first; // min-heap
    }
};

/// Simple hash for GridIndex → int64 key
inline int64_t encodeIndex(int x, int y, int z, int ny, int nz)
{
    return static_cast<int64_t>(x) * static_cast<int64_t>(ny) * static_cast<int64_t>(nz)
         + static_cast<int64_t>(y) * static_cast<int64_t>(nz)
         + static_cast<int64_t>(z);
}

} // namespace jps_detail

// =============================================================================
// JPS3D class
// =============================================================================

/**
 * @brief 3D path planner using A* with 26-connectivity on an occupancy grid.
 *
 * Usage:
 *   JPS3D planner(grid);
 *   if (planner.search(start_world, goal_world))
 *   {
 *       auto path = planner.getPath();
 *   }
 */
class JPS3D
{
public:
    /**
     * @brief Construct a planner bound to the given occupancy grid.
     *
     * The grid reference must remain valid for the lifetime of this planner.
     */
    explicit JPS3D(const OccupancyGrid3D& grid)
        : grid_(grid)
    {}

    /**
     * @brief Search for a collision-free path from start to goal.
     *
     * @param start  World-space start position.
     * @param goal   World-space goal position.
     * @return       true if a path was found.
     */
    bool search(const Eigen::Vector3d& start, const Eigen::Vector3d& goal)
    {
        using namespace jps_detail;

        path_.clear();

        // Convert world → grid
        int sx, sy, sz, gx, gy, gz;
        grid_.worldToGrid(start, sx, sy, sz);
        grid_.worldToGrid(goal,  gx, gy, gz);

        // Validity checks
        if (!grid_.inBounds(sx, sy, sz) || !grid_.inBounds(gx, gy, gz))
            return false;

        if (grid_.isOccupied(sx, sy, sz) || grid_.isOccupied(gx, gy, gz))
            return false;

        // Trivial case: start == goal
        if (sx == gx && sy == gy && sz == gz)
        {
            path_.push_back(grid_.gridToWorld(sx, sy, sz));
            return true;
        }

        int nx = grid_.getNx();
        int ny = grid_.getNy();
        int nz = grid_.getNz();

        // ------------------------------------------------------------------
        // A* data structures
        // ------------------------------------------------------------------
        // Open set: priority queue of (f_cost, node_storage_index)
        std::priority_queue<
            std::pair<double, int>,
            std::vector<std::pair<double, int> >,
            NodeCompare> open_queue;

        // Closed set: grid_key → node_storage_index
        std::unordered_map<int64_t, int> closed_set;
        // Open set lookup: grid_key → node_storage_index
        std::unordered_map<int64_t, int> open_set;

        // Node storage (avoids pointer invalidation)
        std::vector<Node> nodes;
        nodes.reserve(4096);

        // Start node
        Node start_node;
        start_node.idx = GridIndex(sx, sy, sz);
        start_node.g = 0.0;
        start_node.f = euclideanDist(sx, sy, sz, gx, gy, gz);
        start_node.parent = -1;
        nodes.push_back(start_node);

        int64_t start_key = encodeIndex(sx, sy, sz, ny, nz);
        open_set[start_key] = 0;
        open_queue.push(std::make_pair(start_node.f, 0));

        // 26-connectivity offsets
        static const int offsets[26][3] = {
            // 6 face-neighbours
            { 1,  0,  0}, {-1,  0,  0},
            { 0,  1,  0}, { 0, -1,  0},
            { 0,  0,  1}, { 0,  0, -1},
            // 12 edge-neighbours
            { 1,  1,  0}, { 1, -1,  0}, {-1,  1,  0}, {-1, -1,  0},
            { 1,  0,  1}, { 1,  0, -1}, {-1,  0,  1}, {-1,  0, -1},
            { 0,  1,  1}, { 0,  1, -1}, { 0, -1,  1}, { 0, -1, -1},
            // 8 corner-neighbours
            { 1,  1,  1}, { 1,  1, -1}, { 1, -1,  1}, { 1, -1, -1},
            {-1,  1,  1}, {-1,  1, -1}, {-1, -1,  1}, {-1, -1, -1}
        };

        // Step costs: face=1.0, edge=sqrt(2), corner=sqrt(3)
        static const double step_costs[26] = {
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
            1.41421356, 1.41421356, 1.41421356, 1.41421356,
            1.41421356, 1.41421356, 1.41421356, 1.41421356,
            1.41421356, 1.41421356, 1.41421356, 1.41421356,
            1.73205081, 1.73205081, 1.73205081, 1.73205081,
            1.73205081, 1.73205081, 1.73205081, 1.73205081
        };

        bool found = false;
        int goal_node_idx = -1;

        // Limit iterations to prevent unbounded search
        const int MAX_ITER = nx * ny * nz;
        int iter = 0;

        while (!open_queue.empty() && iter < MAX_ITER)
        {
            iter++;
            std::pair<double, int> top = open_queue.top();
            open_queue.pop();
            int cur_idx = top.second;

            const Node& cur = nodes[cur_idx];
            int64_t cur_key = encodeIndex(cur.idx.x, cur.idx.y, cur.idx.z, ny, nz);

            // Skip if already in closed set (stale entry in priority queue)
            if (closed_set.count(cur_key)) continue;
            closed_set[cur_key] = cur_idx;

            // Goal reached?
            if (cur.idx.x == gx && cur.idx.y == gy && cur.idx.z == gz)
            {
                found = true;
                goal_node_idx = cur_idx;
                break;
            }

            // Expand neighbours
            for (int ni = 0; ni < 26; ni++)
            {
                int nx_i = cur.idx.x + offsets[ni][0];
                int ny_i = cur.idx.y + offsets[ni][1];
                int nz_i = cur.idx.z + offsets[ni][2];

                if (!grid_.inBounds(nx_i, ny_i, nz_i)) continue;
                if (grid_.isOccupied(nx_i, ny_i, nz_i)) continue;

                // For diagonal moves, check intermediate voxels to avoid
                // cutting corners through occupied voxels
                if (!isMoveLegal(cur.idx.x, cur.idx.y, cur.idx.z,
                                 offsets[ni][0], offsets[ni][1], offsets[ni][2]))
                    continue;

                int64_t nb_key = encodeIndex(nx_i, ny_i, nz_i, ny, nz);
                if (closed_set.count(nb_key)) continue;

                double new_g = cur.g + step_costs[ni] * grid_.getResolution();
                double h = euclideanDist(nx_i, ny_i, nz_i, gx, gy, gz) * grid_.getResolution();
                double new_f = new_g + h;

                // Check if this neighbour is already in open set with a better cost
                typename std::unordered_map<int64_t, int>::iterator open_it = open_set.find(nb_key);
                if (open_it != open_set.end())
                {
                    Node& existing = nodes[open_it->second];
                    if (new_g < existing.g)
                    {
                        existing.g = new_g;
                        existing.f = new_f;
                        existing.parent = cur_idx;
                        open_queue.push(std::make_pair(new_f, open_it->second));
                    }
                }
                else
                {
                    Node nb_node;
                    nb_node.idx = GridIndex(nx_i, ny_i, nz_i);
                    nb_node.g = new_g;
                    nb_node.f = new_f;
                    nb_node.parent = cur_idx;
                    int nb_storage_idx = static_cast<int>(nodes.size());
                    nodes.push_back(nb_node);
                    open_set[nb_key] = nb_storage_idx;
                    open_queue.push(std::make_pair(new_f, nb_storage_idx));
                }
            }
        }

        if (!found) return false;

        // ------------------------------------------------------------------
        // Reconstruct path
        // ------------------------------------------------------------------
        std::vector<GridIndex> grid_path;
        int idx = goal_node_idx;
        while (idx >= 0)
        {
            grid_path.push_back(nodes[idx].idx);
            idx = nodes[idx].parent;
        }
        std::reverse(grid_path.begin(), grid_path.end());

        // Convert to world coordinates
        std::vector<Eigen::Vector3d> raw_path;
        raw_path.reserve(grid_path.size());
        for (size_t i = 0; i < grid_path.size(); i++)
        {
            raw_path.push_back(grid_.gridToWorld(
                grid_path[i].x, grid_path[i].y, grid_path[i].z));
        }

        // Use actual start/goal world positions (more precise than voxel centres)
        if (!raw_path.empty()) raw_path.front() = start;
        if (raw_path.size() > 1) raw_path.back() = goal;

        // Post-process: simplify collinear / redundant waypoints
        path_ = simplifyPath(raw_path);

        return true;
    }

    /**
     * @brief Return the path found by the last search() call.
     *
     * @return Vector of world-space waypoints from start to goal.
     *         Empty if search() was not called or failed.
     */
    const std::vector<Eigen::Vector3d>& getPath() const { return path_; }

    /**
     * @brief Return a mutable reference to the last computed path.
     */
    std::vector<Eigen::Vector3d>& getPath() { return path_; }

    /**
     * @brief Simplify a path by removing waypoints that are collinear (or
     *        nearly so) by testing direct line-of-sight using the grid.
     *
     * Uses greedy ray-casting: from current start waypoint, advance as far as
     * possible while maintaining line-of-sight. If the direct ray is clear,
     * skip intermediate waypoints.
     *
     * @param input  Input path (ordered world-space waypoints).
     * @return       Simplified path.
     */
    std::vector<Eigen::Vector3d> simplifyPath(
        const std::vector<Eigen::Vector3d>& input) const
    {
        if (input.size() <= 2) return input;

        std::vector<Eigen::Vector3d> result;
        result.push_back(input.front());

        size_t i = 0;
        while (i < input.size() - 1)
        {
            // Find furthest waypoint j > i such that i→j has line of sight
            size_t j = i + 1;
            for (size_t k = input.size() - 1; k > i + 1; k--)
            {
                if (grid_.isSegmentFree(input[i], input[k]))
                {
                    j = k;
                    break;
                }
            }
            result.push_back(input[j]);
            i = j;
        }

        return result;
    }

private:
    const OccupancyGrid3D& grid_;
    std::vector<Eigen::Vector3d> path_;

    /// Euclidean distance between two grid cells (in voxel units)
    double euclideanDist(int x1, int y1, int z1,
                         int x2, int y2, int z2) const
    {
        double dx = static_cast<double>(x2 - x1);
        double dy = static_cast<double>(y2 - y1);
        double dz = static_cast<double>(z2 - z1);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * @brief Check if moving from (x,y,z) by (dx,dy,dz) is legal.
     *
     * For diagonal moves, all intermediate face/edge voxels must be free
     * to prevent cutting corners through obstacles.
     */
    bool isMoveLegal(int x, int y, int z, int dx, int dy, int dz) const
    {
        int nz_count = (dx != 0 ? 1 : 0) + (dy != 0 ? 1 : 0) + (dz != 0 ? 1 : 0);

        if (nz_count == 1)
        {
            // Face move: always legal if target is free (already checked)
            return true;
        }
        else if (nz_count == 2)
        {
            // Edge move: check the two face neighbours
            if (dx != 0 && dy != 0)
            {
                if (grid_.isOccupied(x + dx, y, z)) return false;
                if (grid_.isOccupied(x, y + dy, z)) return false;
            }
            else if (dx != 0 && dz != 0)
            {
                if (grid_.isOccupied(x + dx, y, z)) return false;
                if (grid_.isOccupied(x, y, z + dz)) return false;
            }
            else // dy != 0 && dz != 0
            {
                if (grid_.isOccupied(x, y + dy, z)) return false;
                if (grid_.isOccupied(x, y, z + dz)) return false;
            }
            return true;
        }
        else // nz_count == 3: corner move
        {
            // Corner move: all 3 face + 3 edge neighbours must be free
            if (grid_.isOccupied(x + dx, y, z)) return false;
            if (grid_.isOccupied(x, y + dy, z)) return false;
            if (grid_.isOccupied(x, y, z + dz)) return false;
            if (grid_.isOccupied(x + dx, y + dy, z)) return false;
            if (grid_.isOccupied(x + dx, y, z + dz)) return false;
            if (grid_.isOccupied(x, y + dy, z + dz)) return false;
            return true;
        }
    }
};

} // namespace uav_planning

#endif // UAV_JPS3D_HPP
