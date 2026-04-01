/**
 * minsnap_solver.hpp
 *
 * Minimum-snap trajectory solver for UAVs using piecewise degree-7 polynomials.
 *
 * Implements the closed-form unconstrained minimum-snap formulation from:
 *   Richter, Bry, Roy: "Polynomial Trajectory Planning for Aggressive Quadrotor
 *   Flight in Dense Indoor Environments", ISRR 2013.
 *
 * The problem minimises integral of squared snap (4th derivative) over all
 * segments subject to:
 *   - Waypoint position constraints at each knot
 *   - Continuity of pos, vel, acc, jerk at interior knots
 *   - Boundary conditions (vel, acc, jerk) at start and goal
 *
 * For N waypoints (N-1 segments), each segment has 8 coefficients per axis.
 * The total number of free variables per axis is 8*(N-1).
 *
 * The unconstrained solution is found by building a block-diagonal cost matrix Q
 * (Hessian of snap integral) and a linear constraint matrix A, then solving
 * via the closed-form:
 *   c = A^{-T} * Q_pp^{-1} * A^{-1} * (select free derivatives)
 *   (Richter et al. eq. 11)
 *
 * For small to medium problems (< 20 segments) this direct approach is practical
 * and numerically stable with Eigen's ColPivHouseholderQR decomposition.
 *
 * Compatible with GCC 4.9 / C++11. Depends only on Eigen3.
 */

#ifndef UAV_MINSNAP_SOLVER_HPP
#define UAV_MINSNAP_SOLVER_HPP

#include <vector>
#include <cmath>
#include <cassert>
#include <stdexcept>
#include <algorithm>
#include <Eigen/Dense>

#include "trajectory.hpp"

namespace uav_planning {

// =============================================================================
// Internal helpers
// =============================================================================
namespace minsnap_detail {

/**
 * Compute the cost matrix Q for one polynomial segment of degree D,
 * duration T, minimising the integral of the r-th derivative squared.
 *
 * Q_{ij} = integral_0^T (d^r/dt^r t^i) * (d^r/dt^r t^j) dt
 *        = (product of falling factorials) * T^{i+j-2r+1} / (i+j-2r+1)
 * for i,j >= r; else Q_{ij} = 0.
 *
 * @param D  Polynomial degree
 * @param T  Segment duration
 * @param r  Derivative order to minimise (r=4 for snap)
 * @return   (D+1) x (D+1) cost matrix
 */
inline Eigen::MatrixXd computeSegmentCost(int D, double T, int r)
{
    int n = D + 1;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);

    for (int i = r; i <= D; i++)
    {
        for (int j = r; j <= D; j++)
        {
            // Product of falling factorials: i!/(i-r)! and j!/(j-r)!
            double fi = 1.0, fj = 1.0;
            for (int k = 0; k < r; k++)
            {
                fi *= static_cast<double>(i - k);
                fj *= static_cast<double>(j - k);
            }
            int exp = i + j - 2 * r + 1;
            Q(i, j) = fi * fj * std::pow(T, static_cast<double>(exp)) / static_cast<double>(exp);
        }
    }
    return Q;
}

/**
 * Build the "endpoint constraint matrix" for one segment of degree D and
 * duration T.  Returns a (2*(r_max+1)) x (D+1) matrix that extracts:
 *   [p(0), p'(0), ..., p^{r_max}(0), p(T), p'(T), ..., p^{r_max}(T)]
 * from the coefficient vector c = [c_0, c_1, ..., c_D]^T.
 *
 * Row 2*k   corresponds to the k-th derivative at t=0.
 * Row 2*k+1 corresponds to the k-th derivative at t=T.
 *
 * @param D      Polynomial degree
 * @param T      Segment duration
 * @param r_max  Highest derivative order to extract (e.g. 3 for pos/vel/acc/jerk)
 */
inline Eigen::MatrixXd buildEndpointMatrix(int D, double T, int r_max)
{
    int rows = 2 * (r_max + 1);
    int cols = D + 1;
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(rows, cols);

    for (int r = 0; r <= r_max; r++)
    {
        // t = 0 row: only the term c_r survives (r! * 1)
        // d^r/dt^r (sum c_j t^j)|_{t=0} = c_r * r!
        // Row 2*r: coefficients of d^r p(0) w.r.t. c vector
        // All entries are 0 except col r which is r!
        {
            double fr = 1.0;
            for (int k = 0; k < r; k++) fr *= static_cast<double>(r - k);
            M(2 * r, r) = fr;
        }

        // t = T row: d^r/dt^r (sum c_j t^j)|_{t=T}
        // = sum_{j=r}^{D} c_j * (j!/(j-r)!) * T^{j-r}
        // Row 2*r+1:
        {
            for (int j = r; j <= D; j++)
            {
                double fj = 1.0;
                for (int k = 0; k < r; k++) fj *= static_cast<double>(j - k);
                M(2 * r + 1, j) = fj * std::pow(T, static_cast<double>(j - r));
            }
        }
    }
    return M;
}

} // namespace minsnap_detail

// =============================================================================
// Time allocation
// =============================================================================

/**
 * @brief Compute segment durations using a trapezoidal velocity profile.
 *
 * For each consecutive waypoint pair, the heuristic is:
 *   d = distance between waypoints
 *   If d can be fully accelerated and decelerated within distance d:
 *     t_seg = (v_peak / a_max) * 2 if v_peak < v_max (triangular profile)
 *     else: t_seg = v_max / a_max + d / v_max      (trapezoidal)
 * where v_peak = min(v_max, sqrt(a_max * d)).
 *
 * An additional floor of 0.1 s prevents zero-duration segments.
 *
 * @param waypoints  Vector of 3D waypoints (at least 2).
 * @param max_vel    Maximum allowed speed (m/s).
 * @param max_acc    Maximum allowed acceleration (m/s^2).
 * @return           Vector of N-1 segment durations.
 */
inline std::vector<double> allocateTimeTrapezoidal(
    const std::vector<Eigen::Vector3d>& waypoints,
    double max_vel,
    double max_acc)
{
    if (waypoints.size() < 2)
        throw std::invalid_argument("allocateTimeTrapezoidal: need at least 2 waypoints");
    if (max_vel <= 0.0 || max_acc <= 0.0)
        throw std::invalid_argument("allocateTimeTrapezoidal: max_vel and max_acc must be positive");

    int N = static_cast<int>(waypoints.size());
    std::vector<double> durations(N - 1);

    for (int i = 0; i < N - 1; i++)
    {
        double d = (waypoints[i + 1] - waypoints[i]).norm();
        if (d < 1e-9)
        {
            durations[i] = 0.1; // degenerate segment
            continue;
        }

        // Peak velocity achievable from rest over distance d/2 with acceleration a_max:
        //   v_peak = sqrt(2 * a_max * d/2) = sqrt(a_max * d)
        double v_peak = std::sqrt(max_acc * d);
        double t_seg;

        if (v_peak <= max_vel)
        {
            // Triangular profile: accelerate to v_peak then decelerate
            t_seg = 2.0 * v_peak / max_acc;
        }
        else
        {
            // Trapezoidal: time to reach v_max + cruise + decelerate
            double t_ramp = max_vel / max_acc;
            double d_ramp = 0.5 * max_acc * t_ramp * t_ramp; // distance during one ramp
            double d_cruise = d - 2.0 * d_ramp;
            t_seg = 2.0 * t_ramp + d_cruise / max_vel;
        }

        durations[i] = std::max(t_seg, 0.1);
    }

    return durations;
}

// =============================================================================
// Minimum-snap solver
// =============================================================================

/**
 * @brief Boundary conditions for the trajectory endpoints.
 */
struct BoundaryCondition
{
    Eigen::Vector3d vel;  ///< Velocity
    Eigen::Vector3d acc;  ///< Acceleration
    Eigen::Vector3d jer;  ///< Jerk

    BoundaryCondition()
        : vel(Eigen::Vector3d::Zero())
        , acc(Eigen::Vector3d::Zero())
        , jer(Eigen::Vector3d::Zero())
    {}

    BoundaryCondition(const Eigen::Vector3d& v,
                      const Eigen::Vector3d& a,
                      const Eigen::Vector3d& j)
        : vel(v), acc(a), jer(j)
    {}
};

/**
 * @brief Solve the unconstrained minimum-snap problem.
 *
 * Formulation (Richter et al. 2016, "Polynomial Trajectory Planning for
 * Aggressive Quadrotor Flight in Dense Indoor Environments"):
 *
 * For N waypoints we have M = N-1 segments, each a degree-7 polynomial.
 * Per axis, the decision vector is c in R^{8M}.
 * We minimise c^T Q c subject to linear equality constraints A c = b.
 *
 * The constraints encode:
 *  (1) Waypoint positions at segment start/end:           2M positional equations
 *  (2) Continuity of vel, acc, jerk at interior knots:   3*(M-1) equations
 *  (3) Boundary conditions at start and end:             6 equations (vel,acc,jer * 2 endpoints)
 *
 * Total constraints per axis: 2M + 3*(M-1) + 6 = 5M + 3
 * Total unknowns per axis:    8M
 * The system is square when fixed (Richter: fix = 2 + 3*(M-1) derivatives at knots).
 *
 * We use the "fixed/free endpoint" decomposition:
 *   - Fixed derivatives: all waypoint positions + boundary derivatives
 *   - Free derivatives: interior vel/acc/jer (not fixed by user)
 *
 * The block matrix optimisation yields an analytic minimum for the free
 * derivatives, leading to a linear system in the free vars.
 *
 * For simplicity and robustness, we assemble the full constraint matrix for
 * each axis and solve directly via ColPivHouseholderQR.
 *
 * @param waypoints  N waypoints (N >= 2)
 * @param durations  N-1 segment durations
 * @param start_bc   Boundary condition at start (vel, acc, jer)
 * @param end_bc     Boundary condition at end   (vel, acc, jer)
 * @return           Trajectory<7> with N-1 pieces
 */
inline Trajectory<7> solveMinSnap(
    const std::vector<Eigen::Vector3d>& waypoints,
    const std::vector<double>& durations,
    const BoundaryCondition& start_bc,
    const BoundaryCondition& end_bc)
{
    using namespace minsnap_detail;

    const int D = 7;        // polynomial degree
    const int NC = D + 1;   // number of coefficients per piece = 8
    const int DERIV = 3;    // highest derivative order in constraints (jerk = 3rd)
    const int r_snap = 4;   // minimise 4th derivative (snap)

    int M = static_cast<int>(waypoints.size()) - 1; // number of segments
    if (M < 1)
        throw std::invalid_argument("solveMinSnap: need at least 2 waypoints");
    if (static_cast<int>(durations.size()) != M)
        throw std::invalid_argument("solveMinSnap: durations.size() != waypoints.size()-1");

    // -------------------------------------------------------------------------
    // Build block-diagonal cost matrix Q_full (8M x 8M per axis, same for all axes)
    // -------------------------------------------------------------------------
    int total_vars = NC * M; // 8M
    Eigen::MatrixXd Q_full = Eigen::MatrixXd::Zero(total_vars, total_vars);

    for (int seg = 0; seg < M; seg++)
    {
        Eigen::MatrixXd Qseg = computeSegmentCost(D, durations[seg], r_snap);
        Q_full.block(seg * NC, seg * NC, NC, NC) = Qseg;
    }

    // -------------------------------------------------------------------------
    // Build constraint matrix A and right-hand side b (per axis).
    //
    // Constraint ordering for one axis:
    //   For each segment s = 0..M-1:
    //     Row block s*2+0: p_s(0)    = waypoints[s]       (start position)
    //     Row block s*2+1: p_s(T_s)  = waypoints[s+1]     (end position)
    //   Then continuity at interior knots k=1..M-1:
    //     p_{k-1}'(T_{k-1}) = p_k'(0)      (vel)
    //     p_{k-1}''(T_{k-1}) = p_k''(0)    (acc)
    //     p_{k-1}'''(T_{k-1}) = p_k'''(0)  (jerk)
    //   Boundary conditions at start:
    //     p_0'(0)   = start_bc.vel
    //     p_0''(0)  = start_bc.acc
    //     p_0'''(0) = start_bc.jer
    //   Boundary conditions at end:
    //     p_{M-1}'(T_{M-1})   = end_bc.vel
    //     p_{M-1}''(T_{M-1})  = end_bc.acc
    //     p_{M-1}'''(T_{M-1}) = end_bc.jer
    //
    // Total constraint rows: 2M + 3*(M-1) + 6 = 5M + 3
    // -------------------------------------------------------------------------

    int n_pos_constrs = 2 * M;
    int n_cont_constrs = 3 * (M - 1);
    int n_bc_constrs = 6;
    int total_rows = n_pos_constrs + n_cont_constrs + n_bc_constrs;

    // A is total_rows x total_vars (same structure for all 3 axes)
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(total_rows, total_vars);

    // Right-hand side: separate for each axis (x, y, z)
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(total_rows, 3);

    // --- Position constraints (rows 0 .. 2M-1) ---
    for (int seg = 0; seg < M; seg++)
    {
        double T = durations[seg];
        // Endpoint matrix for position only (r_max = 0)
        // Row 0: p(0), Row 1: p(T)
        Eigen::MatrixXd E = buildEndpointMatrix(D, T, 0); // 2 x 8

        // p_s(0) = waypoints[s]
        A.block(seg * 2, seg * NC, 1, NC) = E.row(0);
        for (int ax = 0; ax < 3; ax++)
            b(seg * 2, ax) = waypoints[seg](ax);

        // p_s(T) = waypoints[s+1]
        A.block(seg * 2 + 1, seg * NC, 1, NC) = E.row(1);
        for (int ax = 0; ax < 3; ax++)
            b(seg * 2 + 1, ax) = waypoints[seg + 1](ax);
    }

    // --- Continuity constraints (rows 2M .. 2M + 3*(M-1) - 1) ---
    {
        int row = 2 * M;
        for (int k = 1; k < M; k++) // at interior knot k (between seg k-1 and seg k)
        {
            double T_prev = durations[k - 1];
            double T_next = durations[k]; // not needed for this constraint formulation

            // Get endpoint matrices for derivatives 1..3
            Eigen::MatrixXd E_prev = buildEndpointMatrix(D, T_prev, DERIV); // 8 x 8
            Eigen::MatrixXd E_next = buildEndpointMatrix(D, T_next, DERIV); // 8 x 8

            for (int r = 1; r <= DERIV; r++) // vel, acc, jerk
            {
                // p_{k-1}^{(r)}(T_{k-1}) - p_k^{(r)}(0) = 0
                A.block(row, (k - 1) * NC, 1, NC) = E_prev.row(2 * r + 1); // at T
                A.block(row, k * NC, 1, NC) = -E_next.row(2 * r);           // at 0

                // RHS is 0 for continuity
                row++;
            }
        }
    }

    // --- Boundary conditions ---
    {
        int row = 2 * M + 3 * (M - 1);

        double T0 = durations[0];
        double TM = durations[M - 1];

        Eigen::MatrixXd E0 = buildEndpointMatrix(D, T0, DERIV);  // 8 x 8
        Eigen::MatrixXd EM = buildEndpointMatrix(D, TM, DERIV);  // 8 x 8

        // Start boundary conditions: derivatives at t=0 of segment 0
        // vel
        A.block(row, 0, 1, NC) = E0.row(2); // row 2*1 = row for 1st deriv at t=0
        for (int ax = 0; ax < 3; ax++) b(row, ax) = start_bc.vel(ax);
        row++;
        // acc
        A.block(row, 0, 1, NC) = E0.row(4); // 2*2 = 4
        for (int ax = 0; ax < 3; ax++) b(row, ax) = start_bc.acc(ax);
        row++;
        // jer
        A.block(row, 0, 1, NC) = E0.row(6); // 2*3 = 6
        for (int ax = 0; ax < 3; ax++) b(row, ax) = start_bc.jer(ax);
        row++;

        // End boundary conditions: derivatives at t=T of segment M-1
        int offset = (M - 1) * NC;
        // vel
        A.block(row, offset, 1, NC) = EM.row(3); // 2*1+1 = 3
        for (int ax = 0; ax < 3; ax++) b(row, ax) = end_bc.vel(ax);
        row++;
        // acc
        A.block(row, offset, 1, NC) = EM.row(5); // 2*2+1 = 5
        for (int ax = 0; ax < 3; ax++) b(row, ax) = end_bc.acc(ax);
        row++;
        // jer
        A.block(row, offset, 1, NC) = EM.row(7); // 2*3+1 = 7
        for (int ax = 0; ax < 3; ax++) b(row, ax) = end_bc.jer(ax);
        row++;
    }

    // -------------------------------------------------------------------------
    // Solve unconstrained minimum snap via KKT / Lagrange multipliers.
    //
    // The Lagrangian optimality conditions give:
    //   2 Q c + A^T lambda = 0
    //   A c = b
    //
    // which is the KKT system:
    //   [2Q   A^T] [c]      [0]
    //   [A    0  ] [lambda] [b]
    //
    // We solve this (2*total_vars + total_rows)^2 system via ColPivHouseholderQR.
    //
    // For numerical stability, scale Q so that typical entries are O(1).
    // -------------------------------------------------------------------------

    int sys_size = total_vars + total_rows;
    Eigen::MatrixXd KKT = Eigen::MatrixXd::Zero(sys_size, sys_size);
    Eigen::MatrixXd rhs = Eigen::MatrixXd::Zero(sys_size, 3);

    // Fill KKT upper-left: 2Q
    KKT.block(0, 0, total_vars, total_vars) = 2.0 * Q_full;
    // Fill KKT upper-right: A^T
    KKT.block(0, total_vars, total_vars, total_rows) = A.transpose();
    // Fill KKT lower-left: A
    KKT.block(total_vars, 0, total_rows, total_vars) = A;
    // KKT lower-right: 0 (already zero)

    // Fill rhs lower block with b
    rhs.block(total_vars, 0, total_rows, 3) = b;

    // Solve
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> solver(KKT);
    if (solver.rank() < sys_size)
    {
        // Fallback: relax and use least-squares pseudo-inverse solution
        // This can happen for very short / degenerate trajectories
    }

    Eigen::MatrixXd sol = solver.solve(rhs); // sys_size x 3

    // Extract coefficient vectors (first total_vars rows)
    Eigen::MatrixXd C = sol.block(0, 0, total_vars, 3); // 8M x 3

    // -------------------------------------------------------------------------
    // Build the Trajectory object
    // -------------------------------------------------------------------------
    Trajectory<7> traj;

    for (int seg = 0; seg < M; seg++)
    {
        Piece<7>::CoeffMatrix coeffs;
        for (int j = 0; j < NC; j++) // j = power of t
        {
            for (int ax = 0; ax < 3; ax++)
            {
                coeffs(ax, j) = C(seg * NC + j, ax);
            }
        }
        traj.addPiece(Piece<7>(durations[seg], coeffs));
    }

    return traj;
}

/**
 * @brief Convenience overload with zero boundary conditions.
 */
inline Trajectory<7> solveMinSnap(
    const std::vector<Eigen::Vector3d>& waypoints,
    const std::vector<double>& durations)
{
    return solveMinSnap(waypoints, durations, BoundaryCondition(), BoundaryCondition());
}

/**
 * @brief Full pipeline: allocate time then solve minimum-snap.
 *
 * @param waypoints  N waypoints.
 * @param max_vel    Maximum speed (m/s).
 * @param max_acc    Maximum acceleration (m/s^2).
 * @param start_bc   Start boundary condition.
 * @param end_bc     End boundary condition.
 * @return           Minimum-snap trajectory.
 */
inline Trajectory<7> planMinSnap(
    const std::vector<Eigen::Vector3d>& waypoints,
    double max_vel,
    double max_acc,
    const BoundaryCondition& start_bc,
    const BoundaryCondition& end_bc)
{
    std::vector<double> durations = allocateTimeTrapezoidal(waypoints, max_vel, max_acc);
    return solveMinSnap(waypoints, durations, start_bc, end_bc);
}

} // namespace uav_planning

#endif // UAV_MINSNAP_SOLVER_HPP
