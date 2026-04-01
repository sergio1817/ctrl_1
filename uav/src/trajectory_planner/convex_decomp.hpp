/**
 * convex_decomp.hpp
 *
 * Safe Flight Corridor (SFC) construction via axis-aligned convex decomposition.
 *
 * For each line segment of the path, an axis-aligned bounding box (AABB) is
 * grown in free space by expanding outward along each of the 6 axis directions
 * until an obstacle or grid boundary is hit. The box is then shrunk inward by
 * the robot radius to maintain a clearance margin.
 *
 * Each AABB is represented as a ConvexPolyhedron with 6 half-plane constraints:
 *   x >= x_min  →  -x <= -x_min
 *   x <= x_max
 *   y >= y_min  →  -y <= -y_min
 *   y <= y_max
 *   z >= z_min  →  -z <= -z_min
 *   z <= z_max
 *
 * Consecutive polyhedra are ensured to overlap so that the goal of segment i
 * lies inside polyhedron i and polyhedron i+1.
 *
 * This approach is inspired by the SEED algorithm / EGO-Planner corridor
 * construction but is intentionally simplified for embedded use.
 *
 * Compatible with GCC 4.9 / C++11. Depends only on Eigen3.
 */

#ifndef UAV_CONVEX_DECOMP_HPP
#define UAV_CONVEX_DECOMP_HPP

#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <limits>
#include <Eigen/Dense>

#include "occupancy_grid.hpp"

namespace uav_planning {

// =============================================================================
// ConvexPolyhedron
// =============================================================================

/**
 * @brief Axis-aligned convex polyhedron represented as Ax <= b.
 *
 * The A matrix has one row per half-plane constraint.
 * For an AABB we have 6 constraints:
 *   [-1  0  0]       [-x_min]
 *   [ 1  0  0]       [ x_max]
 *   [ 0 -1  0]  x <= [-y_min]
 *   [ 0  1  0]       [ y_max]
 *   [ 0  0 -1]       [-z_min]
 *   [ 0  0  1]       [ z_max]
 */
struct ConvexPolyhedron
{
    Eigen::MatrixXd A; ///< n_constraints x 3 matrix of normal vectors
    Eigen::VectorXd b; ///< n_constraints right-hand side

    ConvexPolyhedron() {}

    /**
     * @brief Construct an AABB polyhedron from world-space min/max corners.
     */
    ConvexPolyhedron(const Eigen::Vector3d& min_corner,
                     const Eigen::Vector3d& max_corner)
    {
        A.resize(6, 3);
        b.resize(6);

        // -x <= -x_min  ↔  x >= x_min
        A.row(0) << -1.0,  0.0,  0.0;
        b(0) = -min_corner.x();

        //  x <= x_max
        A.row(1) <<  1.0,  0.0,  0.0;
        b(1) =  max_corner.x();

        // -y <= -y_min
        A.row(2) <<  0.0, -1.0,  0.0;
        b(2) = -min_corner.y();

        //  y <= y_max
        A.row(3) <<  0.0,  1.0,  0.0;
        b(3) =  max_corner.y();

        // -z <= -z_min
        A.row(4) <<  0.0,  0.0, -1.0;
        b(4) = -min_corner.z();

        //  z <= z_max
        A.row(5) <<  0.0,  0.0,  1.0;
        b(5) =  max_corner.z();
    }

    /**
     * @brief Test whether a point is inside this polyhedron (Ax <= b).
     *
     * A small tolerance is added to handle numerical boundary cases.
     *
     * @param point  3D world position.
     * @return       true if the point satisfies all constraints.
     */
    bool contains(const Eigen::Vector3d& point, double tol = 1e-6) const
    {
        Eigen::VectorXd res = A * point - b;
        for (int i = 0; i < res.size(); i++)
        {
            if (res(i) > tol) return false;
        }
        return true;
    }

    /**
     * @brief Shrink the polyhedron inward by @p margin on each face.
     *
     * For an AABB this adds margin to x_min, y_min, z_min and subtracts
     * from x_max, y_max, z_max.
     */
    void inflate(double margin)
    {
        // b(0) = -x_min → increase to shrink: -x_min' = -(x_min + margin) → b(0) -= margin
        // b(1) =  x_max → decrease to shrink: b(1) -= margin
        for (int i = 0; i < b.size(); i++)
        {
            b(i) -= margin; // shrink all constraints inward
        }
    }

    /**
     * @brief Expand (grow) the polyhedron outward by @p margin on each face.
     */
    void deflate(double margin)
    {
        for (int i = 0; i < b.size(); i++)
        {
            b(i) += margin;
        }
    }

    bool isEmpty() const { return A.rows() == 0; }

    /**
     * @brief Return the axis-aligned bounding box as min/max vectors.
     *
     * Only valid for AABB-style polyhedra with the standard 6-constraint layout.
     */
    void getAABB(Eigen::Vector3d& min_corner, Eigen::Vector3d& max_corner) const
    {
        if (b.size() < 6)
        {
            min_corner.setConstant(-std::numeric_limits<double>::infinity());
            max_corner.setConstant( std::numeric_limits<double>::infinity());
            return;
        }
        min_corner.x() = -b(0);
        max_corner.x() =  b(1);
        min_corner.y() = -b(2);
        max_corner.y() =  b(3);
        min_corner.z() = -b(4);
        max_corner.z() =  b(5);
    }
};

// =============================================================================
// SafeFlightCorridor
// =============================================================================

/**
 * @brief Constructs a Safe Flight Corridor (SFC) as a sequence of overlapping
 *        convex polyhedra along a given path.
 *
 * For each path segment, one polyhedron (AABB) is produced. Consecutive
 * polyhedra are grown to ensure that the connecting waypoint lies in both.
 */
class SafeFlightCorridor
{
public:
    SafeFlightCorridor() : built_(false) {}

    /**
     * @brief Build the corridor along the given path.
     *
     * @param path          Ordered world-space waypoints.
     * @param grid          Occupancy grid to check obstacles against.
     * @param robot_radius  Safety clearance to maintain from obstacles (metres).
     * @return              true on success, false if any segment is infeasible.
     */
    bool build(const std::vector<Eigen::Vector3d>& path,
               const OccupancyGrid3D& grid,
               double robot_radius)
    {
        polyhedra_.clear();
        built_ = false;

        if (path.size() < 2)
            return false;

        int n_segs = static_cast<int>(path.size()) - 1;

        for (int seg = 0; seg < n_segs; seg++)
        {
            const Eigen::Vector3d& p_start = path[seg];
            const Eigen::Vector3d& p_end   = path[seg + 1];

            // Build AABB for this segment
            ConvexPolyhedron poly = buildAABBForSegment(p_start, p_end, grid, robot_radius);

            if (poly.isEmpty())
                return false; // segment is in collision even without inflation

            polyhedra_.push_back(poly);
        }

        // Ensure overlap: make sure p_end of segment i is inside both poly[i] and poly[i+1]
        // by expanding the shared boundary slightly if needed.
        ensureOverlap(path);

        built_ = true;
        return true;
    }

    /**
     * @brief Access the computed polyhedra.
     */
    const std::vector<ConvexPolyhedron>& getPolyhedra() const { return polyhedra_; }
    std::vector<ConvexPolyhedron>& getPolyhedra() { return polyhedra_; }

    bool isBuilt() const { return built_; }

    int size() const { return static_cast<int>(polyhedra_.size()); }

private:
    std::vector<ConvexPolyhedron> polyhedra_;
    bool built_;

    /**
     * @brief Build an AABB around the line segment [p_start, p_end] by
     *        expanding outward until hitting occupied voxels.
     *
     * The seed box is the tight AABB of the segment endpoints. Each of the
     * 6 faces is then pushed outward (one voxel at a time) as long as the
     * expanded slab contains only free voxels, then shrunk by robot_radius.
     */
    ConvexPolyhedron buildAABBForSegment(
        const Eigen::Vector3d& p_start,
        const Eigen::Vector3d& p_end,
        const OccupancyGrid3D& grid,
        double robot_radius) const
    {
        double res = grid.getResolution();

        // Seed AABB: tight box around the segment (with small margin)
        double x_min = std::min(p_start.x(), p_end.x()) - res * 0.5;
        double x_max = std::max(p_start.x(), p_end.x()) + res * 0.5;
        double y_min = std::min(p_start.y(), p_end.y()) - res * 0.5;
        double y_max = std::max(p_start.y(), p_end.y()) + res * 0.5;
        double z_min = std::min(p_start.z(), p_end.z()) - res * 0.5;
        double z_max = std::max(p_start.z(), p_end.z()) + res * 0.5;

        // Iteratively expand in each direction
        expandFace(grid, x_min, y_min, z_min, x_max, y_max, z_max,
                   -1, 0, 0, x_min); // expand -x
        expandFace(grid, x_min, y_min, z_min, x_max, y_max, z_max,
                    1, 0, 0, x_max); // expand +x
        expandFace(grid, x_min, y_min, z_min, x_max, y_max, z_max,
                    0,-1, 0, y_min); // expand -y
        expandFace(grid, x_min, y_min, z_min, x_max, y_max, z_max,
                    0, 1, 0, y_max); // expand +y
        expandFace(grid, x_min, y_min, z_min, x_max, y_max, z_max,
                    0, 0,-1, z_min); // expand -z
        expandFace(grid, x_min, y_min, z_min, x_max, y_max, z_max,
                    0, 0, 1, z_max); // expand +z

        // Shrink by robot_radius to maintain clearance
        x_min += robot_radius;
        x_max -= robot_radius;
        y_min += robot_radius;
        y_max -= robot_radius;
        z_min += robot_radius;
        z_max -= robot_radius;

        // If the box collapsed, return empty
        if (x_min >= x_max || y_min >= y_max || z_min >= z_max)
            return ConvexPolyhedron();

        return ConvexPolyhedron(
            Eigen::Vector3d(x_min, y_min, z_min),
            Eigen::Vector3d(x_max, y_max, z_max));
    }

    /**
     * @brief Expand one face of the AABB in the given normal direction until
     *        hitting an obstacle or the grid boundary.
     *
     * @param grid         Occupancy grid.
     * @param x_min,y_min,z_min,x_max,y_max,z_max  Current AABB (modified in-place via face_val).
     * @param nx,ny,nz     Direction of expansion (+1 or -1 per axis, one is nonzero).
     * @param face_val     Reference to the face coordinate being expanded.
     */
    void expandFace(const OccupancyGrid3D& grid,
                    double x_min, double y_min, double z_min,
                    double x_max, double y_max, double z_max,
                    int nx_dir, int ny_dir, int nz_dir,
                    double& face_val) const
    {
        double res = grid.getResolution();
        const int MAX_STEPS = 200; // hard limit to prevent runaway expansion

        for (int step = 0; step < MAX_STEPS; step++)
        {
            double candidate = face_val + static_cast<double>(nx_dir + ny_dir + nz_dir) * res;

            // Check if the new slab is free
            double slab_x_min = (nx_dir < 0) ? candidate : x_min;
            double slab_x_max = (nx_dir > 0) ? candidate : x_max;
            double slab_y_min = (ny_dir < 0) ? candidate : y_min;
            double slab_y_max = (ny_dir > 0) ? candidate : y_max;
            double slab_z_min = (nz_dir < 0) ? candidate : z_min;
            double slab_z_max = (nz_dir > 0) ? candidate : z_max;

            if (!isSlabFree(grid, slab_x_min, slab_y_min, slab_z_min,
                                   slab_x_max, slab_y_max, slab_z_max))
                break;

            // Check grid world bounds
            Eigen::Vector3d pt(candidate, candidate, candidate);
            double new_face = face_val + static_cast<double>(nx_dir + ny_dir + nz_dir) * res;

            // Simple bounds check: don't go beyond grid world extents
            Eigen::Vector3d test_point;
            if (nx_dir != 0)      test_point = Eigen::Vector3d(new_face, (y_min+y_max)*0.5, (z_min+z_max)*0.5);
            else if (ny_dir != 0) test_point = Eigen::Vector3d((x_min+x_max)*0.5, new_face, (z_min+z_max)*0.5);
            else                  test_point = Eigen::Vector3d((x_min+x_max)*0.5, (y_min+y_max)*0.5, new_face);

            if (!grid.inBoundsWorld(test_point)) break;

            face_val = new_face;
        }
    }

    /**
     * @brief Check whether all voxels in an axis-aligned slab are free.
     *
     * Samples the slab at resolution-spaced grid points.
     */
    bool isSlabFree(const OccupancyGrid3D& grid,
                    double x_min, double y_min, double z_min,
                    double x_max, double y_max, double z_max) const
    {
        // Get grid indices for the slab corners
        int ix0, iy0, iz0, ix1, iy1, iz1;
        grid.worldToGrid(Eigen::Vector3d(x_min, y_min, z_min), ix0, iy0, iz0);
        grid.worldToGrid(Eigen::Vector3d(x_max, y_max, z_max), ix1, iy1, iz1);

        ix0 = std::max(ix0, 0);
        iy0 = std::max(iy0, 0);
        iz0 = std::max(iz0, 0);
        ix1 = std::min(ix1, grid.getNx() - 1);
        iy1 = std::min(iy1, grid.getNy() - 1);
        iz1 = std::min(iz1, grid.getNz() - 1);

        for (int ix = ix0; ix <= ix1; ix++)
            for (int iy = iy0; iy <= iy1; iy++)
                for (int iz = iz0; iz <= iz1; iz++)
                    if (grid.isOccupied(ix, iy, iz))
                        return false;

        return true;
    }

    /**
     * @brief Ensure consecutive polyhedra overlap at shared waypoints.
     *
     * For each interior waypoint path[i] (i = 1..n_segs-1), it should be
     * contained in both poly[i-1] and poly[i]. If it is not, expand the
     * adjacent polyhedra slightly to include it.
     */
    void ensureOverlap(const std::vector<Eigen::Vector3d>& path)
    {
        int n = static_cast<int>(polyhedra_.size());
        if (n < 2) return;

        for (int i = 0; i < n - 1; i++)
        {
            const Eigen::Vector3d& wp = path[i + 1]; // shared waypoint

            // Ensure wp is inside poly[i]
            expandPolyToContain(polyhedra_[i], wp);

            // Ensure wp is inside poly[i+1]
            expandPolyToContain(polyhedra_[i + 1], wp);
        }
    }

    /**
     * @brief Expand the AABB-style polyhedron (in-place) to contain @p point.
     *
     * Adjusts the 6 faces if the point lies outside any of them.
     */
    void expandPolyToContain(ConvexPolyhedron& poly,
                             const Eigen::Vector3d& point) const
    {
        if (poly.b.size() < 6) return;

        // b(0) = -x_min: -x <= b(0) → x >= -b(0). If point.x < -b(0), expand: b(0) = -point.x
        if (-poly.b(0) > point.x()) poly.b(0) = -point.x() + 1e-6;
        // b(1) = x_max: x <= b(1). If point.x > b(1), expand: b(1) = point.x
        if (poly.b(1) < point.x())  poly.b(1) = point.x()  + 1e-6;
        // b(2) = -y_min
        if (-poly.b(2) > point.y()) poly.b(2) = -point.y() + 1e-6;
        // b(3) = y_max
        if (poly.b(3) < point.y())  poly.b(3) = point.y()  + 1e-6;
        // b(4) = -z_min
        if (-poly.b(4) > point.z()) poly.b(4) = -point.z() + 1e-6;
        // b(5) = z_max
        if (poly.b(5) < point.z())  poly.b(5) = point.z()  + 1e-6;
    }
};

} // namespace uav_planning

#endif // UAV_CONVEX_DECOMP_HPP
