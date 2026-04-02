/*
 * gcopter_wrapper.hpp
 *
 * Thin wrapper around GCOPTER library (Zhepei Wang, MIT License).
 * Converts between our local planning types and GCOPTER's types.
 *
 * GCOPTER provides:
 *   - MINCO_S3NU: sparse 3rd-order MINCO -> Trajectory<5> (degree 5)
 *   - GCOPTER_PolytopeSFC: trajectory optimization with polytope SFC
 *   - Trajectory<D>: piecewise polynomial trajectory (D = degree)
 *
 * MINCO_S3NU is a "minimum control" formulation that minimizes
 * the integral of jerk-squared, producing degree-5 polynomials.
 * This is different from our own min-snap solver (degree 7).
 * Both are valid trajectory representations; the GCOPTER backend
 * is an alternative with different trade-offs (sparser, faster,
 * but lower derivative continuity).
 *
 * C++11 compatible (GCC 4.9). Header-only.
 */

#ifndef GCOPTER_WRAPPER_HPP
#define GCOPTER_WRAPPER_HPP

#include "corridor_qp.hpp"  // for ConvexPolyhedron, AABBCorridor

// GCOPTER headers (all header-only, Eigen-only, no ROS)
#include "gcopter/trajectory.hpp"
#include "gcopter/minco.hpp"

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace uav_planning {

// GCOPTER MINCO produces degree-5 trajectories
typedef Trajectory<5> GCOPTERTrajectory;
typedef Piece<5> GCOPTERPiece;

// Configuration for GCOPTER-based planning
struct GCOPTERConfig {
    double max_vel;     // maximum velocity [m/s]
    double max_acc;     // maximum acceleration [m/s^2]
    double weight_time; // time penalty weight (rho in GCOPTER)

    GCOPTERConfig()
        : max_vel(2.0), max_acc(5.0), weight_time(100.0) {}
};


// ============================================================
// Convert our AABB corridors to GCOPTER's H-representation
// GCOPTER uses Eigen::MatrixX4d: each row = [a, b, c, d]
// representing a*x + b*y + c*z + d <= 0
// ============================================================
inline Eigen::MatrixX4d aabbToHPolyhedron(const AABBCorridor &box) {
    Eigen::MatrixX4d H(6, 4);
    //  x <= hi_x  =>  1*x + 0*y + 0*z - hi_x <= 0
    H.row(0) <<  1,  0,  0, -box.hi.x();
    H.row(1) << -1,  0,  0,  box.lo.x();
    H.row(2) <<  0,  1,  0, -box.hi.y();
    H.row(3) <<  0, -1,  0,  box.lo.y();
    H.row(4) <<  0,  0,  1, -box.hi.z();
    H.row(5) <<  0,  0, -1,  box.lo.z();
    return H;
}


// ============================================================
// Plan using GCOPTER's MINCO backend
//
// MINCO_S3NU solves a minimum-jerk problem producing degree-5
// polynomial segments. The Trajectory<5> result provides
// getPos(t), getVel(t), getAcc(t), getJer(t), getTotalDuration().
//
// For full optimization (velocity/acceleration constraints, polytope SFC),
// use the GCOPTER_PolytopeSFC class directly.
// ============================================================
inline bool planGCOPTER(
    const std::vector<Eigen::Vector3d> &waypoints,
    const std::vector<ConvexPolyhedron> &corridors,
    double max_vel,
    double max_acc,
    GCOPTERTrajectory &result)
{
    const int N = static_cast<int>(waypoints.size());
    if (N < 2) return false;

    const int M = N - 1;  // number of segments
    (void)corridors;  // corridors are used by the full optimizer, not direct MINCO

    // Time allocation: trapezoidal velocity profile heuristic
    std::vector<double> durations(M);
    for (int i = 0; i < M; ++i) {
        double dist = (waypoints[i + 1] - waypoints[i]).norm();
        double t_acc = max_vel / max_acc;  // time to accelerate to max vel
        double t_seg;
        if (dist < max_vel * t_acc) {
            // Short segment: triangular profile
            t_seg = 2.0 * std::sqrt(dist / max_acc);
        } else {
            // Long segment: trapezoidal profile
            t_seg = t_acc + dist / max_vel;
        }
        if (t_seg < 0.3) t_seg = 0.3;
        durations[i] = t_seg;
    }

    // Build boundary conditions: [pos, vel, acc] as 3x3 matrices (column-major)
    Eigen::Matrix3d headPVA;
    headPVA.col(0) = waypoints[0];
    headPVA.col(1) = Eigen::Vector3d::Zero();
    headPVA.col(2) = Eigen::Vector3d::Zero();

    Eigen::Matrix3d tailPVA;
    tailPVA.col(0) = waypoints[N - 1];
    tailPVA.col(1) = Eigen::Vector3d::Zero();
    tailPVA.col(2) = Eigen::Vector3d::Zero();

    // Build the MINCO problem
    minco::MINCO_S3NU minco_solver;
    minco_solver.setConditions(headPVA, tailPVA, M);

    // Set intermediate waypoints
    Eigen::Matrix3Xd inPts(3, M - 1);
    for (int i = 1; i < N - 1; ++i) {
        inPts.col(i - 1) = waypoints[i];
    }

    Eigen::VectorXd T(M);
    for (int i = 0; i < M; ++i) {
        T(i) = durations[i];
    }

    minco_solver.setParameters(inPts, T);

    // Extract the trajectory
    GCOPTERTrajectory traj;
    minco_solver.getTrajectory(traj);

    result = traj;
    return true;
}


// ============================================================
// Simplified wrapper: plan without corridors (just waypoints)
// ============================================================
inline bool planGCOPTERSimple(
    const std::vector<Eigen::Vector3d> &waypoints,
    double max_vel,
    double max_acc,
    GCOPTERTrajectory &result)
{
    std::vector<ConvexPolyhedron> empty_corridors;
    return planGCOPTER(waypoints, empty_corridors, max_vel, max_acc, result);
}

}  // namespace uav_planning

#endif  // GCOPTER_WRAPPER_HPP
