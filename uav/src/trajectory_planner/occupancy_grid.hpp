/**
 * occupancy_grid.hpp
 *
 * 3D occupancy grid for obstacle representation.
 *
 * Voxel storage uses a flat std::vector<uint8_t> with row-major indexing:
 *   index = ix * (ny * nz) + iy * nz + iz
 *
 * World coordinates are mapped to grid indices by:
 *   ix = floor((pos.x - origin.x) / resolution)
 *
 * Compatible with GCC 4.9 / C++11. Depends only on Eigen3.
 */

#ifndef UAV_OCCUPANCY_GRID_HPP
#define UAV_OCCUPANCY_GRID_HPP

#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <Eigen/Dense>

namespace uav_planning {

/**
 * @brief 3D occupancy grid representing obstacles in a workspace.
 *
 * Provides efficient voxel lookup by world coordinate or grid index.
 * Each voxel is stored as uint8_t: 0 = free, 1 = occupied.
 */
class OccupancyGrid3D
{
public:
    /**
     * @brief Construct an empty (all-free) occupancy grid.
     *
     * @param origin     World coordinate of the (0,0,0) voxel corner.
     * @param nx         Number of voxels in x direction.
     * @param ny         Number of voxels in y direction.
     * @param nz         Number of voxels in z direction.
     * @param resolution Side length of each cubic voxel (metres).
     */
    OccupancyGrid3D(const Eigen::Vector3d& origin,
                    int nx, int ny, int nz,
                    double resolution)
        : origin_(origin)
        , nx_(nx)
        , ny_(ny)
        , nz_(nz)
        , resolution_(resolution)
        , data_(static_cast<size_t>(nx) * static_cast<size_t>(ny) * static_cast<size_t>(nz), 0u)
    {
        if (nx <= 0 || ny <= 0 || nz <= 0)
            throw std::invalid_argument("OccupancyGrid3D: dimensions must be positive");
        if (resolution <= 0.0)
            throw std::invalid_argument("OccupancyGrid3D: resolution must be positive");
    }

    // -------------------------------------------------------------------------
    // Grid dimensions and parameters
    // -------------------------------------------------------------------------
    int getNx() const { return nx_; }
    int getNy() const { return ny_; }
    int getNz() const { return nz_; }
    double getResolution() const { return resolution_; }
    const Eigen::Vector3d& getOrigin() const { return origin_; }

    // -------------------------------------------------------------------------
    // Occupancy mutation
    // -------------------------------------------------------------------------

    /**
     * @brief Mark a voxel as occupied. No-op if indices are out of bounds.
     */
    void setOccupied(int ix, int iy, int iz)
    {
        if (!inBounds(ix, iy, iz)) return;
        data_[index(ix, iy, iz)] = 1u;
    }

    /**
     * @brief Mark a voxel as free. No-op if indices are out of bounds.
     */
    void setFree(int ix, int iy, int iz)
    {
        if (!inBounds(ix, iy, iz)) return;
        data_[index(ix, iy, iz)] = 0u;
    }

    // -------------------------------------------------------------------------
    // Occupancy queries
    // -------------------------------------------------------------------------

    /**
     * @brief Query occupancy by grid indices. Returns true (occupied) for
     *        out-of-bounds queries (treat outside grid as obstacle).
     */
    bool isOccupied(int ix, int iy, int iz) const
    {
        if (!inBounds(ix, iy, iz)) return true; // out of bounds = occupied
        return data_[index(ix, iy, iz)] != 0u;
    }

    /**
     * @brief Query occupancy by world-space position.
     *
     * @param pos  World coordinate (metres).
     * @return     true if the voxel at that position is occupied (or out of bounds).
     */
    bool isOccupied(const Eigen::Vector3d& pos) const
    {
        int ix, iy, iz;
        worldToGrid(pos, ix, iy, iz);
        return isOccupied(ix, iy, iz);
    }

    // -------------------------------------------------------------------------
    // Coordinate conversion
    // -------------------------------------------------------------------------

    /**
     * @brief Convert world coordinates to grid indices.
     *        Indices may be out of bounds; caller should check with inBounds().
     */
    void worldToGrid(const Eigen::Vector3d& pos, int& ix, int& iy, int& iz) const
    {
        ix = static_cast<int>(std::floor((pos.x() - origin_.x()) / resolution_));
        iy = static_cast<int>(std::floor((pos.y() - origin_.y()) / resolution_));
        iz = static_cast<int>(std::floor((pos.z() - origin_.z()) / resolution_));
    }

    /**
     * @brief Convert grid indices to world coordinates (centre of voxel).
     */
    Eigen::Vector3d gridToWorld(int ix, int iy, int iz) const
    {
        return Eigen::Vector3d(
            origin_.x() + (ix + 0.5) * resolution_,
            origin_.y() + (iy + 0.5) * resolution_,
            origin_.z() + (iz + 0.5) * resolution_);
    }

    // -------------------------------------------------------------------------
    // Obstacle primitives
    // -------------------------------------------------------------------------

    /**
     * @brief Mark all voxels whose centres lie within @p radius of @p center as occupied.
     *
     * Iterates only over the bounding box of the sphere for efficiency.
     */
    void addSphereObstacle(const Eigen::Vector3d& center, double radius)
    {
        // Grid indices of sphere bounding box
        int ix_min, iy_min, iz_min, ix_max, iy_max, iz_max;

        worldToGrid(center - Eigen::Vector3d(radius, radius, radius), ix_min, iy_min, iz_min);
        worldToGrid(center + Eigen::Vector3d(radius, radius, radius), ix_max, iy_max, iz_max);

        // Clamp to grid bounds
        ix_min = std::max(ix_min, 0);
        iy_min = std::max(iy_min, 0);
        iz_min = std::max(iz_min, 0);
        ix_max = std::min(ix_max, nx_ - 1);
        iy_max = std::min(iy_max, ny_ - 1);
        iz_max = std::min(iz_max, nz_ - 1);

        double r2 = radius * radius;

        for (int ix = ix_min; ix <= ix_max; ix++)
        {
            for (int iy = iy_min; iy <= iy_max; iy++)
            {
                for (int iz = iz_min; iz <= iz_max; iz++)
                {
                    Eigen::Vector3d voxelCenter = gridToWorld(ix, iy, iz);
                    double dx = voxelCenter.x() - center.x();
                    double dy = voxelCenter.y() - center.y();
                    double dz = voxelCenter.z() - center.z();
                    if (dx*dx + dy*dy + dz*dz <= r2)
                    {
                        data_[index(ix, iy, iz)] = 1u;
                    }
                }
            }
        }
    }

    /**
     * @brief Mark an axis-aligned box as occupied.
     *
     * @param min_corner  World-space minimum corner of the box.
     * @param max_corner  World-space maximum corner of the box.
     */
    void addBoxObstacle(const Eigen::Vector3d& min_corner,
                        const Eigen::Vector3d& max_corner)
    {
        int ix_min, iy_min, iz_min, ix_max, iy_max, iz_max;
        worldToGrid(min_corner, ix_min, iy_min, iz_min);
        worldToGrid(max_corner, ix_max, iy_max, iz_max);

        ix_min = std::max(ix_min, 0);
        iy_min = std::max(iy_min, 0);
        iz_min = std::max(iz_min, 0);
        ix_max = std::min(ix_max, nx_ - 1);
        iy_max = std::min(iy_max, ny_ - 1);
        iz_max = std::min(iz_max, nz_ - 1);

        for (int ix = ix_min; ix <= ix_max; ix++)
            for (int iy = iy_min; iy <= iy_max; iy++)
                for (int iz = iz_min; iz <= iz_max; iz++)
                    data_[index(ix, iy, iz)] = 1u;
    }

    /**
     * @brief Reset all voxels to free.
     */
    void clear()
    {
        std::fill(data_.begin(), data_.end(), 0u);
    }

    // -------------------------------------------------------------------------
    // Bounds check
    // -------------------------------------------------------------------------
    bool inBounds(int ix, int iy, int iz) const
    {
        return ix >= 0 && ix < nx_ &&
               iy >= 0 && iy < ny_ &&
               iz >= 0 && iz < nz_;
    }

    /**
     * @brief Check if a world position is inside the grid bounds.
     */
    bool inBoundsWorld(const Eigen::Vector3d& pos) const
    {
        int ix, iy, iz;
        worldToGrid(pos, ix, iy, iz);
        return inBounds(ix, iy, iz);
    }

    /**
     * @brief Ray-casting: check if the line segment from @p start to @p end
     *        passes through any occupied voxel (Bresenham 3D traversal).
     *
     * @return true if the segment is collision-free.
     */
    bool isSegmentFree(const Eigen::Vector3d& start,
                       const Eigen::Vector3d& end) const
    {
        // Walk along segment in steps of resolution/2
        Eigen::Vector3d diff = end - start;
        double length = diff.norm();
        if (length < 1e-9) return !isOccupied(start);

        int steps = static_cast<int>(std::ceil(length / (resolution_ * 0.5)));
        Eigen::Vector3d step = diff / static_cast<double>(steps);

        for (int i = 0; i <= steps; i++)
        {
            Eigen::Vector3d pt = start + step * static_cast<double>(i);
            if (isOccupied(pt)) return false;
        }
        return true;
    }

    /**
     * @brief Return the total number of voxels.
     */
    size_t totalVoxels() const
    {
        return data_.size();
    }

private:
    Eigen::Vector3d origin_;
    int nx_, ny_, nz_;
    double resolution_;
    std::vector<uint8_t> data_;

    // Row-major flat index: x is outermost, z is innermost
    size_t index(int ix, int iy, int iz) const
    {
        return static_cast<size_t>(ix) * static_cast<size_t>(ny_) * static_cast<size_t>(nz_)
             + static_cast<size_t>(iy) * static_cast<size_t>(nz_)
             + static_cast<size_t>(iz);
    }
};

} // namespace uav_planning

#endif // UAV_OCCUPANCY_GRID_HPP
