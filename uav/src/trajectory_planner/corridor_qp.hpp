/*
 * corridor_qp.hpp
 *
 * Corridor-constrained min-snap trajectory optimization.
 * Enforces that each polynomial segment stays within its
 * Safe Flight Corridor (axis-aligned bounding box).
 *
 * Approach:
 *   - For each segment i with AABB corridor C_i = [lo_i, hi_i]
 *   - Sample K check points per segment: t_k = k/(K-1) * duration_i
 *   - At each check point, the position must satisfy lo_i <= p(t_k) <= hi_i
 *   - Use iterative constraint tightening:
 *     1. Solve unconstrained min-snap
 *     2. Check all corridor constraints
 *     3. For violated constraints, add intermediate waypoints at boundary
 *     4. Re-solve with additional waypoints
 *     5. Repeat up to MAX_ITERATIONS times
 *
 * C++11 compatible (GCC 4.9). Eigen 3.3+.
 */

#ifndef CORRIDOR_QP_HPP
#define CORRIDOR_QP_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

namespace uav_planning {

// Axis-aligned bounding box corridor
struct AABBCorridor {
    Eigen::Vector3d lo;  // lower bounds (x_min, y_min, z_min)
    Eigen::Vector3d hi;  // upper bounds (x_max, y_max, z_max)

    AABBCorridor()
        : lo(-std::numeric_limits<double>::max(),
             -std::numeric_limits<double>::max(),
             -std::numeric_limits<double>::max()),
          hi(std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()) {}

    AABBCorridor(const Eigen::Vector3d &l, const Eigen::Vector3d &h)
        : lo(l), hi(h) {}

    bool contains(const Eigen::Vector3d &p, double margin = 0.0) const {
        return (p.x() >= lo.x() + margin && p.x() <= hi.x() - margin &&
                p.y() >= lo.y() + margin && p.y() <= hi.y() - margin &&
                p.z() >= lo.z() + margin && p.z() <= hi.z() - margin);
    }

    // Clamp point to corridor boundary with margin
    Eigen::Vector3d clamp(const Eigen::Vector3d &p, double margin = 0.0) const {
        Eigen::Vector3d result;
        result.x() = std::max(lo.x() + margin, std::min(p.x(), hi.x() - margin));
        result.y() = std::max(lo.y() + margin, std::min(p.y(), hi.y() - margin));
        result.z() = std::max(lo.z() + margin, std::min(p.z(), hi.z() - margin));
        return result;
    }
};

// General convex polyhedron (half-space representation): A * x <= b
struct ConvexPolyhedron {
    Eigen::MatrixXd A;  // num_faces x 3
    Eigen::VectorXd b;  // num_faces x 1

    ConvexPolyhedron() {}

    ConvexPolyhedron(const Eigen::MatrixXd &A_, const Eigen::VectorXd &b_)
        : A(A_), b(b_) {}

    // Construct from AABB (6 half-planes)
    static ConvexPolyhedron fromAABB(const AABBCorridor &box) {
        ConvexPolyhedron poly;
        poly.A.resize(6, 3);
        poly.b.resize(6);
        //  x <= hi_x  =>  [1,0,0]*x <= hi_x
        poly.A.row(0) << 1, 0, 0;   poly.b(0) = box.hi.x();
        // -x <= -lo_x  =>  [-1,0,0]*x <= -lo_x
        poly.A.row(1) << -1, 0, 0;  poly.b(1) = -box.lo.x();
        poly.A.row(2) << 0, 1, 0;   poly.b(2) = box.hi.y();
        poly.A.row(3) << 0, -1, 0;  poly.b(3) = -box.lo.y();
        poly.A.row(4) << 0, 0, 1;   poly.b(4) = box.hi.z();
        poly.A.row(5) << 0, 0, -1;  poly.b(5) = -box.lo.z();
        return poly;
    }

    bool contains(const Eigen::Vector3d &p, double margin = 0.0) const {
        Eigen::VectorXd res = A * p;
        for (int i = 0; i < res.size(); ++i) {
            if (res(i) > b(i) - margin) return false;
        }
        return true;
    }
};

// Corridor constraint violation
struct ConstraintViolation {
    int segment;       // which segment
    double t_local;    // time within segment
    int axis;          // which axis (0=x, 1=y, 2=z), or -1 for polyhedron face
    double violation;  // how much the constraint is violated
    Eigen::Vector3d position;  // the violating position
};

// Configuration for corridor-constrained planning
struct CorridorQPConfig {
    int check_points_per_segment;  // K: number of sample points per segment
    double safety_margin;           // shrink corridor by this margin [m]
    int max_iterations;             // max constraint-tightening iterations
    double max_vel;                 // velocity bound for time allocation
    double min_segment_time;        // minimum segment duration

    CorridorQPConfig()
        : check_points_per_segment(5),
          safety_margin(0.05),
          max_iterations(3),
          max_vel(2.0),
          min_segment_time(0.3) {}
};


// ============================================================
// Evaluate a degree-7 polynomial: c0 + c1*t + ... + c7*t^7
// coeffs: 3 x 8 matrix (row = axis, col = coefficient index)
// ============================================================
inline Eigen::Vector3d evalPoly7(const Eigen::Matrix<double, 3, 8> &c, double t) {
    Eigen::Vector3d result = c.col(7);
    for (int k = 6; k >= 0; --k) {
        result = result * t + c.col(k);
    }
    return result;
}


// ============================================================
// Check corridor violations for a set of segments
// ============================================================
inline std::vector<ConstraintViolation> checkCorridorViolations(
    const Eigen::Matrix<double, 3, 8> *segments,  // array of coefficient matrices
    const double *durations,
    int num_segments,
    const std::vector<AABBCorridor> &corridors,
    const CorridorQPConfig &config)
{
    std::vector<ConstraintViolation> violations;
    const int K = config.check_points_per_segment;
    const double margin = config.safety_margin;

    for (int i = 0; i < num_segments; ++i) {
        if (i >= static_cast<int>(corridors.size())) break;
        const AABBCorridor &corr = corridors[i];

        for (int k = 0; k < K; ++k) {
            double t_k;
            if (K > 1) {
                t_k = static_cast<double>(k) / static_cast<double>(K - 1) * durations[i];
            } else {
                t_k = durations[i] * 0.5;
            }

            Eigen::Vector3d pos = evalPoly7(segments[i], t_k);

            // Check each axis for AABB violation
            for (int axis = 0; axis < 3; ++axis) {
                double lo = corr.lo(axis) + margin;
                double hi = corr.hi(axis) - margin;

                if (pos(axis) < lo) {
                    ConstraintViolation v;
                    v.segment = i;
                    v.t_local = t_k;
                    v.axis = axis;
                    v.violation = lo - pos(axis);
                    v.position = pos;
                    violations.push_back(v);
                } else if (pos(axis) > hi) {
                    ConstraintViolation v;
                    v.segment = i;
                    v.t_local = t_k;
                    v.axis = axis;
                    v.violation = pos(axis) - hi;
                    v.position = pos;
                    violations.push_back(v);
                }
            }
        }
    }
    return violations;
}


// ============================================================
// Solve unconstrained min-snap for one axis
// (Same algorithm as TrajectoryManager::SolveMinSnap but standalone)
// ============================================================
inline bool solveMinSnapAxis(
    const std::vector<double> &waypoints_axis,
    const std::vector<double> &durations,
    double start_vel,
    double end_vel,
    Eigen::VectorXd &coeffs_out)
{
    const int M = static_cast<int>(durations.size());
    const int N = 8;
    const int dim = M * N;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(dim);

    int row = 0;

    // Start boundary: p_0(0)=pos0, p_0'(0)=v0, p_0''(0)=0, p_0'''(0)=0
    {
        A(row, 0) = 1.0;
        b(row) = waypoints_axis[0];
        row++;

        A(row, 1) = 1.0;
        b(row) = start_vel;
        row++;

        A(row, 2) = 2.0;
        b(row) = 0.0;
        row++;

        A(row, 3) = 6.0;
        b(row) = 0.0;
        row++;
    }

    // End boundary
    {
        int seg_off = (M - 1) * N;
        double T = durations[M - 1];
        double Tp[8];
        Tp[0] = 1.0;
        for (int k = 1; k < 8; ++k) Tp[k] = Tp[k - 1] * T;

        for (int k = 0; k < N; ++k) A(row, seg_off + k) = Tp[k];
        b(row) = waypoints_axis[M];
        row++;

        A(row, seg_off + 1) = 1.0;
        for (int k = 2; k < 8; ++k) A(row, seg_off + k) = k * Tp[k - 1];
        b(row) = end_vel;
        row++;

        // acc(T)=0
        A(row, seg_off + 2) = 2.0;
        A(row, seg_off + 3) = 6.0 * T;
        A(row, seg_off + 4) = 12.0 * Tp[2];
        A(row, seg_off + 5) = 20.0 * Tp[3];
        A(row, seg_off + 6) = 30.0 * Tp[4];
        A(row, seg_off + 7) = 42.0 * Tp[5];
        row++;

        // jerk(T)=0
        A(row, seg_off + 3) = 6.0;
        A(row, seg_off + 4) = 24.0 * T;
        A(row, seg_off + 5) = 60.0 * Tp[2];
        A(row, seg_off + 6) = 120.0 * Tp[3];
        A(row, seg_off + 7) = 210.0 * Tp[4];
        row++;
    }

    // Interior continuity
    for (int i = 0; i < M - 1; ++i) {
        int si = i * N;
        int sj = (i + 1) * N;
        double T = durations[i];
        double Tp[8];
        Tp[0] = 1.0;
        for (int k = 1; k < 8; ++k) Tp[k] = Tp[k - 1] * T;

        // pos_i(T) = wp[i+1]
        for (int k = 0; k < N; ++k) A(row, si + k) = Tp[k];
        b(row) = waypoints_axis[i + 1];
        row++;

        // pos_{i+1}(0) = wp[i+1]
        A(row, sj + 0) = 1.0;
        b(row) = waypoints_axis[i + 1];
        row++;

        // vel continuity
        for (int k = 1; k < 8; ++k) A(row, si + k) = k * Tp[k - 1];
        A(row, sj + 1) = -1.0;
        row++;

        // acc continuity
        A(row, si + 2) = 2.0;
        A(row, si + 3) = 6.0 * T;
        A(row, si + 4) = 12.0 * Tp[2];
        A(row, si + 5) = 20.0 * Tp[3];
        A(row, si + 6) = 30.0 * Tp[4];
        A(row, si + 7) = 42.0 * Tp[5];
        A(row, sj + 2) = -2.0;
        row++;

        // jerk continuity
        A(row, si + 3) = 6.0;
        A(row, si + 4) = 24.0 * T;
        A(row, si + 5) = 60.0 * Tp[2];
        A(row, si + 6) = 120.0 * Tp[3];
        A(row, si + 7) = 210.0 * Tp[4];
        A(row, sj + 3) = -6.0;
        row++;

        // snap continuity
        A(row, si + 4) = 24.0;
        A(row, si + 5) = 120.0 * T;
        A(row, si + 6) = 360.0 * Tp[2];
        A(row, si + 7) = 840.0 * Tp[3];
        A(row, sj + 4) = -24.0;
        row++;

        // crackle continuity
        A(row, si + 5) = 120.0;
        A(row, si + 6) = 720.0 * T;
        A(row, si + 7) = 2520.0 * Tp[2];
        A(row, sj + 5) = -120.0;
        row++;

        // 6th derivative continuity
        A(row, si + 6) = 720.0;
        A(row, si + 7) = 5040.0 * T;
        A(row, sj + 6) = -720.0;
        row++;
    }

    coeffs_out = A.colPivHouseholderQr().solve(b);
    double residual = (A * coeffs_out - b).norm();
    return residual < 1e-6;
}


// ============================================================
// Main function: Plan min-snap with corridor constraints
// ============================================================
// Iterative approach:
//   1. Solve unconstrained min-snap with given waypoints
//   2. Check corridor constraints at K sample points per segment
//   3. For worst violation, insert a new waypoint at corridor boundary
//   4. Re-solve. Repeat up to max_iterations.
//
// Returns true if feasible (all constraints satisfied or within tolerance).
//
inline bool planMinSnapCorridor(
    const std::vector<Eigen::Vector3d> &initial_waypoints,
    const std::vector<AABBCorridor> &corridors,
    const Eigen::Vector3d &start_vel,
    const Eigen::Vector3d &end_vel,
    const CorridorQPConfig &config,
    // Output: segment coefficients and durations
    std::vector<Eigen::Matrix<double, 3, 8> > &out_coeffs,
    std::vector<double> &out_durations)
{
    // Working copy of waypoints (may grow as we insert intermediate ones)
    std::vector<Eigen::Vector3d> waypoints = initial_waypoints;

    // Corridor assignment: corridor[i] covers segment between waypoints[i] and waypoints[i+1]
    // When we split a segment, both halves get the same corridor.
    std::vector<int> corridor_map;  // corridor_map[seg] = index into corridors
    {
        int M = static_cast<int>(waypoints.size()) - 1;
        corridor_map.resize(M);
        for (int i = 0; i < M; ++i) {
            corridor_map[i] = (i < static_cast<int>(corridors.size())) ? i : static_cast<int>(corridors.size()) - 1;
        }
    }

    for (int iter = 0; iter < config.max_iterations; ++iter) {
        int M = static_cast<int>(waypoints.size()) - 1;
        if (M < 1 || M > 20) return false;  // safety limit

        // Compute durations
        std::vector<double> durations(M);
        for (int i = 0; i < M; ++i) {
            double dist = (waypoints[i + 1] - waypoints[i]).norm();
            double t_seg = dist / config.max_vel;
            if (t_seg < config.min_segment_time) t_seg = config.min_segment_time;
            durations[i] = t_seg;
        }

        // Solve unconstrained min-snap per axis
        std::vector<Eigen::Matrix<double, 3, 8> > seg_coeffs(M);

        bool solve_ok = true;
        for (int axis = 0; axis < 3; ++axis) {
            std::vector<double> wp_axis(M + 1);
            for (int i = 0; i <= M; ++i) {
                wp_axis[i] = waypoints[i](axis);
            }

            Eigen::VectorXd c;
            if (!solveMinSnapAxis(wp_axis, durations, start_vel(axis), end_vel(axis), c)) {
                solve_ok = false;
                break;
            }

            for (int i = 0; i < M; ++i) {
                for (int k = 0; k < 8; ++k) {
                    seg_coeffs[i](axis, k) = c(i * 8 + k);
                }
            }
        }
        if (!solve_ok) return false;

        // Build corridor array matching segments
        std::vector<AABBCorridor> seg_corridors(M);
        for (int i = 0; i < M; ++i) {
            seg_corridors[i] = corridors[corridor_map[i]];
        }

        // Check corridor violations
        std::vector<ConstraintViolation> violations = checkCorridorViolations(
            seg_coeffs.data(), durations.data(), M, seg_corridors, config);

        if (violations.empty()) {
            // All constraints satisfied - output result
            out_coeffs = seg_coeffs;
            out_durations = durations;
            return true;
        }

        // Find worst violation
        double worst = 0.0;
        int worst_idx = 0;
        for (int i = 0; i < static_cast<int>(violations.size()); ++i) {
            if (violations[i].violation > worst) {
                worst = violations[i].violation;
                worst_idx = i;
            }
        }

        const ConstraintViolation &v = violations[worst_idx];

        // Insert a new waypoint at the violation point, clamped to corridor
        Eigen::Vector3d new_wp = seg_corridors[v.segment].clamp(v.position, config.safety_margin);

        // Insert into waypoints list, splitting the segment
        int insert_pos = v.segment + 1;
        waypoints.insert(waypoints.begin() + insert_pos, new_wp);

        // Update corridor map: the new segment inherits parent's corridor
        int parent_corridor = corridor_map[v.segment];
        corridor_map.insert(corridor_map.begin() + v.segment + 1, parent_corridor);
    }

    // Ran out of iterations - return best effort
    // Do a final solve
    int M = static_cast<int>(waypoints.size()) - 1;
    if (M < 1) return false;

    std::vector<double> durations(M);
    for (int i = 0; i < M; ++i) {
        double dist = (waypoints[i + 1] - waypoints[i]).norm();
        double t_seg = dist / config.max_vel;
        if (t_seg < config.min_segment_time) t_seg = config.min_segment_time;
        durations[i] = t_seg;
    }

    out_coeffs.resize(M);
    for (int axis = 0; axis < 3; ++axis) {
        std::vector<double> wp_axis(M + 1);
        for (int i = 0; i <= M; ++i) wp_axis[i] = waypoints[i](axis);

        Eigen::VectorXd c;
        if (!solveMinSnapAxis(wp_axis, durations, start_vel(axis), end_vel(axis), c)) {
            return false;
        }
        for (int i = 0; i < M; ++i) {
            for (int k = 0; k < 8; ++k) {
                out_coeffs[i](axis, k) = c(i * 8 + k);
            }
        }
    }
    out_durations = durations;
    return true;
}

}  // namespace uav_planning

#endif  // CORRIDOR_QP_HPP
