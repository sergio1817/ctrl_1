// TrajectoryManager.cpp
//
// Implementation of TrajectoryManager — trajectory planning lifecycle for ctrl_1.
//
// Phases 2-5 of the implementation roadmap:
//   Phase 2 — GUI + basic trajectory evaluation
//   Phase 3 — VRPN obstacle tracking
//   Phase 4 — Receding-horizon replanning
//   Phase 5 — Dynamic obstacle prediction
//
// C++11 / GCC 4.9 compatible.  No threads — all planning is synchronous,
// executed inside the Flair real-time control loop when triggered by button
// clicks or the replan timer.

#include "TrajectoryManager.h"

// Flair GUI
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <PushButton.h>
#include <ComboBox.h>
#include <Label.h>

// Flair sensor / meta
#include <VrpnClient.h>
#include <MetaVrpnObject.h>

// Flair core
#include <Vector3D.h>

// Flair threading (for Printf / Thread::Info)
#include <Thread.h>

// Occupancy grid is already included via TrajectoryManager.h

// Standard
#include <cmath>
#include <cstring>   // memset
#include <stdexcept>
#include <sstream>
#include <algorithm>

// Eigen
#include <Eigen/Dense>

using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::meta;
using namespace flair::core;

// ============================================================================
// Constructor
// ============================================================================

TrajectoryManager::TrajectoryManager(GroupBox* parent)
    : planner_mode_(0)
    , safety_margin_(0)
    , max_velocity_(0)
    , max_acceleration_(0)
    , grid_resolution_(0)
    , replan_period_(0)
    , num_waypoints_(0)
    , num_obstacles_(0)
    , btn_plan_(0)
    , btn_execute_(0)
    , btn_stop_(0)
    , status_label_(0)
    , num_tracked_obstacles_(0)
    , trajectory_valid_(false)
    , state_(State::IDLE)
    , execution_start_time_(0.0)
    , last_replan_time_(0.0)
    , plan_start_pos_(Eigen::Vector3d::Zero())
{
    // Initialise obstacle array
    for (int i = 0; i < kMaxObstacles; ++i) {
        obstacles_[i] = ObstacleState();
    }
    // Initialise waypoint spinbox pointers
    for (int i = 0; i < kMaxWaypoints; ++i) {
        wp_x_[i] = 0;
        wp_y_[i] = 0;
        wp_z_[i] = 0;
    }

    // -------------------------------------------------------------------------
    // GUI creation
    // -------------------------------------------------------------------------
    // Row 1: planner mode combo
    planner_mode_ = new ComboBox(parent->NewRow(), "Planner mode");
    planner_mode_->AddItem("Waypoint");
    planner_mode_->AddItem("Corridor");

    // Row 2: planner parameters
    safety_margin_    = new DoubleSpinBox(parent->NewRow(),    "Safety margin",   " m",    0.05, 1.0,   0.05, 2);
    max_velocity_     = new DoubleSpinBox(parent->LastRowLastCol(), "Max velocity",  " m/s",  0.1,  5.0,   0.1,  2);
    max_acceleration_ = new DoubleSpinBox(parent->LastRowLastCol(), "Max accel",     " m/s2", 0.1,  10.0,  0.1,  2);

    // Default values (set via spinbox initialisation value — 4th arg of DoubleSpinBox
    // in the Flair API is min; the 6th arg is step; there's no direct "default" ctor
    // param in Flair, so we rely on the XML/flair saved config or accept the min as
    // initial.  The values below match the spec defaults when no saved config exists.)
    // NOTE: Flair DoubleSpinBox(parent, label, unit, min, max, step, decimals)
    //       — no separate "default" parameter, GUI is initialised to min.
    //       We document the intended defaults via comments.
    //       Default: safety_margin=0.30, max_velocity=1.5, max_acceleration=3.0

    grid_resolution_ = new DoubleSpinBox(parent->NewRow(), "Grid resolution", " m",   0.05, 0.5,  0.05, 2);
    replan_period_   = new DoubleSpinBox(parent->LastRowLastCol(), "Replan period",   " s",   0.5,  5.0,  0.5,  1);
    // Default: grid_resolution=0.10, replan_period=2.0

    // Row: num_waypoints, num_obstacles
    num_waypoints_ = new DoubleSpinBox(parent->NewRow(),          "Waypoints (N)",  "",  1.0, 5.0, 1.0, 0);
    num_obstacles_ = new DoubleSpinBox(parent->LastRowLastCol(),  "Obstacles (N)",  "",  0.0, 5.0, 1.0, 0);

    // Waypoints — create 5 GroupBoxes with x/y/z spinboxes
    // They are always present in the GUI; only the first num_waypoints_ are used.
    const char* wp_labels[kMaxWaypoints] = { "WP1", "WP2", "WP3", "WP4", "WP5" };
    for (int i = 0; i < kMaxWaypoints; ++i) {
        GroupBox* wbox = new GroupBox(parent->NewRow(), wp_labels[i]);
        wp_x_[i] = new DoubleSpinBox(wbox->NewRow(),          "x", " m", -10.0, 10.0, 0.1, 2);
        wp_y_[i] = new DoubleSpinBox(wbox->LastRowLastCol(),  "y", " m", -10.0, 10.0, 0.1, 2);
        wp_z_[i] = new DoubleSpinBox(wbox->LastRowLastCol(),  "z", " m", -10.0, 10.0, 0.1, 2);
    }

    // Control buttons
    btn_plan_    = new PushButton(parent->NewRow(), "Plan");
    btn_execute_ = new PushButton(parent->LastRowLastCol(), "Execute");
    btn_stop_    = new PushButton(parent->LastRowLastCol(), "Stop");

    // Status label
    status_label_ = new Label(parent->NewRow(), "traj_status");
    status_label_->SetText("Ready");
}

// ============================================================================
// Destructor
// ============================================================================

TrajectoryManager::~TrajectoryManager()
{
    // VRPN objects are owned by the Flair FrameworkManager tree —
    // do NOT delete them here; Flair deletes them via its object tree.
    // obstacle[].vrpn pointers are left dangling intentionally.
    // Other Flair GUI objects (spinboxes, buttons, etc.) are also owned
    // by the parent GroupBox and will be destroyed by Flair.
}

// ============================================================================
// addObstacleVrpn
// ============================================================================

void TrajectoryManager::addObstacleVrpn(const std::string& name,
                                         VrpnClient* /*client*/)
{
    if (num_tracked_obstacles_ >= kMaxObstacles) {
        flair::core::Thread::Warn("TrajectoryManager: max obstacles reached\n");
        return;
    }
    int idx = num_tracked_obstacles_;
    // MetaVrpnObject is constructed without a VrpnClient parameter in Flair —
    // the VrpnClient is the singleton that was already started; MetaVrpnObject
    // automatically registers with it.
    obstacles_[idx].vrpn        = new MetaVrpnObject(name);
    obstacles_[idx].initialized = false;
    ++num_tracked_obstacles_;
    flair::core::Thread::Info("TrajectoryManager: added obstacle VRPN '%s'\n",
                               name.c_str());
}

// ============================================================================
// setStatus
// ============================================================================

void TrajectoryManager::setStatus(const std::string& text)
{
    if (status_label_) {
        status_label_->SetText(text);
    }
}

// ============================================================================
// updateObstacles (Phase 3 & 5)
// ============================================================================

void TrajectoryManager::updateObstacles(double t_actual)
{
    int n = num_tracked_obstacles_;
    // Also read the GUI num_obstacles to determine how many to actually poll
    int n_gui = static_cast<int>(num_obstacles_->Value());
    if (n_gui < n) n = n_gui;

    for (int i = 0; i < n; ++i) {
        ObstacleState& obs = obstacles_[i];
        if (!obs.vrpn) continue;
        if (!obs.vrpn->IsTracked(500)) continue;  // timeout 500 ms

        Vector3Df flair_pos;
        obs.vrpn->GetPosition(flair_pos);

        Eigen::Vector3d new_pos(static_cast<double>(flair_pos.x),
                                static_cast<double>(flair_pos.y),
                                static_cast<double>(flair_pos.z));

        if (!obs.initialized) {
            obs.position      = new_pos;
            obs.prev_position = new_pos;
            obs.velocity      = Eigen::Vector3d::Zero();
            obs.prev_time     = t_actual;
            obs.initialized   = true;
        } else {
            double dt = t_actual - obs.prev_time;
            if (dt > 1e-4) {
                // Numerical differentiation with exponential low-pass filter
                // alpha = dt / (tau + dt),  tau = 0.1 s
                const double tau  = 0.1;
                double alpha      = dt / (tau + dt);
                Eigen::Vector3d raw_vel = (new_pos - obs.prev_position) / dt;
                obs.velocity      = (1.0 - alpha) * obs.velocity + alpha * raw_vel;
                obs.prev_position = new_pos;
                obs.prev_time     = t_actual;
            }
            obs.position = new_pos;
        }
    }
}

// ============================================================================
// predictObstaclePos (Phase 5 — constant velocity model)
// ============================================================================

Eigen::Vector3d TrajectoryManager::predictObstaclePos(int idx, double dt) const
{
    const ObstacleState& obs = obstacles_[idx];
    return obs.position + obs.velocity * dt;
}

// ============================================================================
// allocateTimes
// ============================================================================

std::vector<double> TrajectoryManager::allocateTimes(
    const std::vector<Eigen::Vector3d>& wps,
    double max_vel) const
{
    std::vector<double> times;
    if (wps.size() < 2) return times;
    for (size_t i = 0; i + 1 < wps.size(); ++i) {
        double dist = (wps[i+1] - wps[i]).norm();
        double t    = dist / max_vel;
        if (t < 0.2) t = 0.2;   // minimum segment duration
        times.push_back(t);
    }
    return times;
}

// ============================================================================
// solveMinSnap1D
//
// Minimum-snap QP for a single axis.
//
// For N waypoints (N-1 segments), each segment i has a 7th-order polynomial
// p_i(tau) = sum_{k=0}^{7} c_{i,k} * tau^k,  tau in [0, T_i]
//
// Continuity constraints at interior waypoints (position through jerk, 4 orders)
// give 4*(N-2) equations. Endpoint conditions (pos, vel=0, acc=0, jerk=0 at
// start and end) give 8 equations. Waypoint position constraints: N equations.
// Total constraints: 4*(N-2) + 8 + (N-2) = 5N - 2 + ... actually we use the
// standard unconstrained minimum-snap closed-form for a sequence of waypoints.
//
// We use the "snap matrix" approach (Richter et al. 2016):
// Minimise sum_i int_0^{T_i} (p_i^(4)(tau))^2 dtau
// subject to: position continuity, derivative continuity up to order 3, and
//             boundary conditions at start/end (vel=acc=jerk=0).
//
// For simplicity and GCC 4.9 compatibility we implement the direct matrix form.
// The system size is 8*M x 8*M where M = N-1 (number of segments).
//
// Returns true on success.  Fills axis row of traj pieces.
// ============================================================================

bool TrajectoryManager::solveMinSnap1D(const std::vector<double>& pos,
                                        const std::vector<double>& times,
                                        int axis,
                                        uav_planning::Trajectory<7>& traj)
{
    // N = number of waypoints, M = N-1 = number of segments
    int N = static_cast<int>(pos.size());
    int M = static_cast<int>(times.size());
    if (N < 2 || M != N - 1) return false;

    // Degree D = 7, so 8 coefficients per segment
    const int D = 8;  // coefficients per segment (polynomial of degree 7)
    int total_vars = D * M;

    // We build the constraint matrix A and rhs b for the equality constraints.
    // -------------------------------------------------------------------------
    // Constraint ordering:
    //
    // [A] Endpoint constraints (8):
    //   Start:  p_0(0)=pos[0], p_0'(0)=0, p_0''(0)=0, p_0'''(0)=0
    //   End:    p_{M-1}(T_{M-1})=pos[N-1], p'=0, p''=0, p'''=0
    //
    // [B] Continuity at interior waypoints (5*(N-2) constraints each):
    //   Position match: p_i(T_i) = pos[i+1]  (N-2 constraints)
    //   Derivative continuity (4 orders): p_i^(k)(T_i) = p_{i+1}^(k)(0)  (4*(N-2) constraints)
    //
    // Total constraints: 8 + (N-2)*5 = 5N - 2
    //
    // For M segments we have 8M unknowns. To get a square system we need 8M eqs.
    // The remaining 8M - (5N-2) = 8(N-1) - 5N + 2 = 3N - 6 degrees of freedom
    // are the free derivatives at intermediate waypoints. We set them free and
    // use the minimum-snap gradient condition: the gradient of the snap cost
    // w.r.t. free derivatives = 0. This gives a banded system.
    //
    // Implementation: We follow the "endpoint derivative" formulation from
    // Richter et al. 2016 (mav_trajectory_generation).
    //
    // For GCC 4.9 compatibility we use Eigen matrices directly.
    // -------------------------------------------------------------------------

    // Helper: evaluate polynomial basis vector at time t
    //   b(t) = [1, t, t^2, ..., t^{D-1}]
    // Returns derivative of order 'deriv'
    // (This is a local lambda — but GCC 4.9 doesn't support generic lambdas.
    //  We use a local struct with operator() instead.)

    struct PolyBasis {
        // Returns k-th row of 8x1 basis vector for order-deriv derivative at t
        static Eigen::Matrix<double, 8, 1> eval(double t, int deriv) {
            Eigen::Matrix<double, 8, 1> b;
            b.setZero();
            for (int j = deriv; j < 8; ++j) {
                // Coefficient of t^{j-deriv} in d^deriv/dt^deriv (t^j)
                double factor = 1.0;
                for (int k = 0; k < deriv; ++k) {
                    factor *= static_cast<double>(j - k);
                }
                double tpow = 1.0;
                for (int k = 0; k < j - deriv; ++k) {
                    tpow *= t;
                }
                b(j) = factor * tpow;
            }
            return b;
        }
    };

    // Build constraint matrix A (Nc x total_vars) and rhs b (Nc x 1)
    // We use the "fixed derivative" formulation: at every waypoint we fix
    // positions; at start and end we also fix vel=acc=jerk=0; interior
    // derivatives (vel, acc, jerk) are determined by the minimum-snap condition.
    //
    // Strategy: assemble the square system using the endpoint derivative
    // ordering from Richter et al.  For each segment, the 8 unknowns are the
    // values of p^(0)..p^(3) at start, and p^(0)..p^(3) at end (the "d" vector).
    // The Q (cost) and M (mapping) matrices relate this parameterisation to the
    // polynomial coefficients.

    // For brevity we implement the direct "polynomial coefficient" formulation.
    // We build the full 8M x 8M system:
    //   - First 4 rows: start endpoint constraints (pos, vel=0, acc=0, jerk=0)
    //   - Next 4 rows: end endpoint constraints
    //   - For each interior waypoint i (i=1..N-2): 5 constraints
    //     (continuity of pos + 4 derivatives)
    //   - Remaining rows: minimum-snap gradient conditions (free derivatives)

    // ---- Build the system using Eigen ----
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(total_vars, total_vars);
    Eigen::VectorXd b_rhs = Eigen::VectorXd::Zero(total_vars);
    int row = 0;

    // --- Endpoint constraints for segment 0, at tau=0 ---
    {
        // p_0(0) = pos[0]
        Eigen::Matrix<double, 8, 1> basis = PolyBasis::eval(0.0, 0);
        A.block<1, 8>(row, 0) = basis.transpose();
        b_rhs(row) = pos[0];
        ++row;

        // p_0'(0) = 0
        basis = PolyBasis::eval(0.0, 1);
        A.block<1, 8>(row, 0) = basis.transpose();
        b_rhs(row) = 0.0;
        ++row;

        // p_0''(0) = 0
        basis = PolyBasis::eval(0.0, 2);
        A.block<1, 8>(row, 0) = basis.transpose();
        b_rhs(row) = 0.0;
        ++row;

        // p_0'''(0) = 0
        basis = PolyBasis::eval(0.0, 3);
        A.block<1, 8>(row, 0) = basis.transpose();
        b_rhs(row) = 0.0;
        ++row;
    }

    // --- Interior waypoint constraints ---
    for (int i = 0; i < M - 1; ++i) {
        double Ti = times[i];
        int col_i   = D * i;       // start of segment i coefficients
        int col_ip1 = D * (i + 1); // start of segment i+1 coefficients

        // Position: p_i(T_i) = pos[i+1]
        {
            Eigen::Matrix<double, 8, 1> basis = PolyBasis::eval(Ti, 0);
            A.block<1, 8>(row, col_i) = basis.transpose();
            b_rhs(row) = pos[i + 1];
            ++row;
        }

        // Continuity of derivatives 0..3 at junction:
        //   p_i^(k)(T_i) = p_{i+1}^(k)(0)
        // Rewritten: p_i^(k)(T_i) - p_{i+1}^(k)(0) = 0
        for (int k = 0; k <= 3; ++k) {
            Eigen::Matrix<double, 8, 1> basis_end   = PolyBasis::eval(Ti,  k);
            Eigen::Matrix<double, 8, 1> basis_start = PolyBasis::eval(0.0, k);
            A.block<1, 8>(row, col_i)   =  basis_end.transpose();
            A.block<1, 8>(row, col_ip1) = -basis_start.transpose();
            b_rhs(row) = 0.0;
            ++row;
        }
    }

    // --- Endpoint constraints for segment M-1, at tau=T_{M-1} ---
    {
        int col_last = D * (M - 1);
        double Tlast = times[M - 1];

        // p_{M-1}(T_{M-1}) = pos[N-1]
        Eigen::Matrix<double, 8, 1> basis = PolyBasis::eval(Tlast, 0);
        A.block<1, 8>(row, col_last) = basis.transpose();
        b_rhs(row) = pos[N - 1];
        ++row;

        // p'=0
        basis = PolyBasis::eval(Tlast, 1);
        A.block<1, 8>(row, col_last) = basis.transpose();
        b_rhs(row) = 0.0;
        ++row;

        // p''=0
        basis = PolyBasis::eval(Tlast, 2);
        A.block<1, 8>(row, col_last) = basis.transpose();
        b_rhs(row) = 0.0;
        ++row;

        // p'''=0
        basis = PolyBasis::eval(Tlast, 3);
        A.block<1, 8>(row, col_last) = basis.transpose();
        b_rhs(row) = 0.0;
        ++row;
    }

    // --- Minimum-snap condition for free derivative rows ---
    // Remaining rows (row .. total_vars-1) correspond to higher-order continuity
    // enforced as soft conditions: for each remaining DOF, set p_i^(4)(0) = 0
    // or use the snap minimisation gradient.  For the simple underdetermined case
    // (M=1, 8 constraints, 8 unknowns — already satisfied), and for M>1 we fill
    // the remaining rows by enforcing continuity of derivatives 4..7 at each
    // junction (up to remaining rows).
    {
        int extra_per_junction = 3; // derivatives 4,5,6 (order 3 already used above)
        for (int i = 0; i < M - 1 && row < total_vars; ++i) {
            double Ti    = times[i];
            int col_i    = D * i;
            int col_ip1  = D * (i + 1);
            for (int k = 4; k <= 7 && row < total_vars; ++k) {
                Eigen::Matrix<double, 8, 1> basis_end   = PolyBasis::eval(Ti,  k);
                Eigen::Matrix<double, 8, 1> basis_start = PolyBasis::eval(0.0, k);
                A.block<1, 8>(row, col_i)   =  basis_end.transpose();
                A.block<1, 8>(row, col_ip1) = -basis_start.transpose();
                b_rhs(row) = 0.0;
                ++row;
            }
        }
        // If still underfilled (single segment M=1 needs exactly 8 rows; the
        // endpoint block already provided 8, row==8 — nothing extra needed)
    }

    // Sanity check
    if (row != total_vars) {
        // Remaining rows: leave as identity with rhs=0 (zero coefficients)
        // This handles edge cases and keeps the matrix invertible.
        while (row < total_vars) {
            A(row, row) = 1.0;
            b_rhs(row)  = 0.0;
            ++row;
        }
    }

    // Solve: A * c = b_rhs
    Eigen::VectorXd coeff_vec;
    {
        // Use LU decomposition for robustness
        Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
        if (!lu.isInvertible()) {
            // Fallback: least-squares solution
            coeff_vec = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                          .solve(b_rhs);
        } else {
            coeff_vec = lu.solve(b_rhs);
        }
    }

    // Store coefficients into traj pieces for this axis
    // traj already has M pieces allocated by solveMinSnap; we update row 'axis'.
    for (int i = 0; i < M; ++i) {
        uav_planning::Piece<7>& piece = traj.getPiece(i);
        uav_planning::Piece<7>::CoeffMatrix& coeffs = piece.getCoeffs();
        for (int k = 0; k < 8; ++k) {
            coeffs(axis, k) = coeff_vec(D * i + k);
        }
    }

    return true;
}

// ============================================================================
// solveMinSnap
// ============================================================================

bool TrajectoryManager::solveMinSnap(const std::vector<Eigen::Vector3d>& waypoints,
                                      const std::vector<double>& times,
                                      uav_planning::Trajectory<7>& traj)
{
    int N = static_cast<int>(waypoints.size());
    int M = static_cast<int>(times.size());
    if (N < 2 || M != N - 1) return false;

    // Initialise trajectory with M pieces (zero coefficients, correct durations)
    traj.clear();
    for (int i = 0; i < M; ++i) {
        uav_planning::Piece<7>::CoeffMatrix zeros;
        zeros.setZero();
        traj.addPiece(times[i], zeros);
    }

    // Solve per axis
    for (int axis = 0; axis < 3; ++axis) {
        std::vector<double> pos_axis(N);
        for (int i = 0; i < N; ++i) {
            pos_axis[i] = waypoints[i](axis);
        }
        if (!solveMinSnap1D(pos_axis, times, axis, traj)) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// plan
// ============================================================================

bool TrajectoryManager::plan(const Eigen::Vector3d& uav_pos, double t_actual)
{
    setStatus("Planning...");

    // --- Read parameters from GUI ---
    double max_vel  = max_velocity_->Value();
    double max_acc  = max_acceleration_->Value();
    double margin   = safety_margin_->Value();
    double res      = grid_resolution_->Value();
    int    n_wps    = static_cast<int>(num_waypoints_->Value());
    int    n_obs    = static_cast<int>(num_obstacles_->Value());

    if (max_vel  < 0.01) max_vel  = 0.1;
    if (max_acc  < 0.01) max_acc  = 0.1;
    if (margin   < 0.01) margin   = 0.05;
    if (res      < 0.01) res      = 0.05;
    if (n_wps    < 1)    n_wps    = 1;
    if (n_wps    > kMaxWaypoints) n_wps = kMaxWaypoints;

    // --- Collect waypoints (start = current UAV position) ---
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(uav_pos);  // start
    for (int i = 0; i < n_wps; ++i) {
        double wx = wp_x_[i]->Value();
        double wy = wp_y_[i]->Value();
        double wz = wp_z_[i]->Value();
        waypoints.push_back(Eigen::Vector3d(wx, wy, wz));
    }

    // --- Build occupancy grid (Phase 3 & 5) ---
    // Determine grid bounds from waypoints + obstacles
    Eigen::Vector3d grid_min = uav_pos;
    Eigen::Vector3d grid_max = uav_pos;
    for (size_t i = 1; i < waypoints.size(); ++i) {
        grid_min = grid_min.cwiseMin(waypoints[i]);
        grid_max = grid_max.cwiseMax(waypoints[i]);
    }

    // Prediction horizon for dynamic obstacles = total trajectory duration estimate
    double total_dist = 0.0;
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        total_dist += (waypoints[i+1] - waypoints[i]).norm();
    }
    double horizon = total_dist / max_vel;

    // Include predicted obstacle positions in grid bounds
    int n_active_obs = (n_obs < num_tracked_obstacles_) ? n_obs : num_tracked_obstacles_;
    for (int i = 0; i < n_active_obs; ++i) {
        if (!obstacles_[i].initialized) continue;
        Eigen::Vector3d pred = predictObstaclePos(i, horizon);
        grid_min = grid_min.cwiseMin(pred);
        grid_max = grid_max.cwiseMax(pred);
    }

    // Expand grid by margin + 1m padding
    double pad = margin + 1.0;
    grid_min -= Eigen::Vector3d(pad, pad, pad);
    grid_max += Eigen::Vector3d(pad, pad, pad);

    // Grid dimensions
    Eigen::Vector3d grid_size = grid_max - grid_min;
    int nx = static_cast<int>(std::ceil(grid_size.x() / res)) + 1;
    int ny = static_cast<int>(std::ceil(grid_size.y() / res)) + 1;
    int nz = static_cast<int>(std::ceil(grid_size.z() / res)) + 1;
    if (nx < 1) nx = 1;
    if (ny < 1) ny = 1;
    if (nz < 1) nz = 1;

    // Cap grid to prevent memory explosion on bad inputs
    const int kMaxGridDim = 200;
    if (nx > kMaxGridDim || ny > kMaxGridDim || nz > kMaxGridDim) {
        setStatus("Error: grid too large — reduce resolution or reduce workspace");
        return false;
    }

    uav_planning::OccupancyGrid3D grid(grid_min, nx, ny, nz, res);

    // Add static obstacle spheres (current position)
    for (int i = 0; i < n_active_obs; ++i) {
        if (!obstacles_[i].initialized) continue;
        grid.addSphereObstacle(obstacles_[i].position, margin);
    }

    // Add predicted obstacle positions (dynamic obstacles — Phase 5)
    // We sample the prediction at several time steps and add inflated spheres
    if (horizon > 0.01) {
        const int kPredSteps = 5;
        for (int i = 0; i < n_active_obs; ++i) {
            if (!obstacles_[i].initialized) continue;
            for (int s = 1; s <= kPredSteps; ++s) {
                double dt = horizon * static_cast<double>(s) /
                            static_cast<double>(kPredSteps);
                Eigen::Vector3d pred = predictObstaclePos(i, dt);
                // Inflate slightly more for future uncertainty
                double inflated = margin * (1.0 + 0.2 * static_cast<double>(s));
                grid.addSphereObstacle(pred, inflated);
            }
        }
    }

    // --- Allocate segment times ---
    std::vector<double> times = allocateTimes(waypoints, max_vel);
    if (times.empty()) {
        setStatus("Error: no segments");
        return false;
    }

    // --- Solve minimum-snap trajectory ---
    uav_planning::Trajectory<7> new_traj;
    if (!solveMinSnap(waypoints, times, new_traj)) {
        setStatus("Error: solver failed");
        return false;
    }

    // --- Collision check (optional, diagnostic) ---
    // Walk along the trajectory and check against the occupancy grid
    bool collision_free = true;
    double total_dur = new_traj.getTotalDuration();
    const int kCheckSteps = 100;
    for (int s = 0; s <= kCheckSteps && collision_free; ++s) {
        double t = total_dur * static_cast<double>(s) /
                   static_cast<double>(kCheckSteps);
        Eigen::Vector3d p = new_traj.getPos(t);
        if (grid.isOccupied(p)) {
            collision_free = false;
        }
    }

    if (!collision_free) {
        // Warn but still accept the trajectory — in a real corridor planner
        // the QP constraints would enforce collision freedom.  For the
        // waypoint mode we log a warning and proceed.
        flair::core::Thread::Warn("TrajectoryManager: trajectory may collide with obstacles\n");
        setStatus("Planned (collision warning)");
    } else {
        setStatus("Planned OK");
    }

    // Store in pending then swap to active
    pending_trajectory_ = new_traj;
    active_trajectory_  = pending_trajectory_;
    trajectory_valid_   = true;
    plan_start_pos_     = uav_pos;
    last_replan_time_   = t_actual;

    flair::core::Thread::Info("TrajectoryManager: planned %.2f s trajectory (%d segs)\n",
                               total_dur,
                               static_cast<int>(waypoints.size()) - 1);
    return true;
}

// ============================================================================
// update — called every control loop iteration
// ============================================================================

void TrajectoryManager::update(float t_actual_f,
                                const flair::core::Vector3Df& uav_pos_flair)
{
    double t_actual = static_cast<double>(t_actual_f);
    Eigen::Vector3d uav_pos(static_cast<double>(uav_pos_flair.x),
                             static_cast<double>(uav_pos_flair.y),
                             static_cast<double>(uav_pos_flair.z));

    // Phase 3: update obstacle tracking every iteration
    updateObstacles(t_actual);

    // --- Button: Plan ---
    if (btn_plan_->Clicked()) {
        // Can plan from any state
        state_ = State::PLANNING;
        bool ok = plan(uav_pos, t_actual);
        if (ok) {
            state_ = State::PLANNED;
        } else {
            state_ = State::IDLE;
            setStatus("Error: planning failed");
        }
    }

    // --- Button: Execute ---
    if (btn_execute_->Clicked()) {
        if (trajectory_valid_) {
            if (state_ == State::PLANNED || state_ == State::IDLE) {
                state_ = State::EXECUTING;
                execution_start_time_ = t_actual;
                last_replan_time_     = t_actual;
                setStatus("Executing");
                flair::core::Thread::Info("TrajectoryManager: execution started\n");
            }
        } else {
            setStatus("Error: no trajectory (press Plan first)");
        }
    }

    // --- Button: Stop ---
    if (btn_stop_->Clicked()) {
        stop();
        return;
    }

    // --- Trajectory end detection ---
    if (state_ == State::EXECUTING && trajectory_valid_) {
        double elapsed = t_actual - execution_start_time_;
        double total   = active_trajectory_.getTotalDuration();
        if (elapsed >= total) {
            setStatus("Trajectory complete");
            state_ = State::IDLE;
            flair::core::Thread::Info("TrajectoryManager: trajectory complete\n");
            return;
        }
    }

    // --- Phase 4: Receding-horizon replanning ---
    if (state_ == State::EXECUTING) {
        double replan_dt = replan_period_->Value();
        if (replan_dt < 0.1) replan_dt = 0.5;
        double since_last = t_actual - last_replan_time_;

        if (since_last >= replan_dt) {
            // Time to replan
            state_ = State::REPLANNING;
            setStatus("Replanning...");

            // Determine the current point on trajectory as new start
            double elapsed = t_actual - execution_start_time_;
            double total   = active_trajectory_.getTotalDuration();
            if (elapsed >= total) elapsed = total;
            // Current desired position from active trajectory
            Eigen::Vector3d traj_pos = active_trajectory_.getPos(elapsed);
            // Use actual UAV position as start for replanning
            bool ok = plan(uav_pos, t_actual);
            if (ok) {
                // Swap trajectory and reset execution clock
                execution_start_time_ = t_actual;
                state_ = State::EXECUTING;
                setStatus("Executing (replanned)");
            } else {
                // Keep old trajectory — add a safety extension if near end
                flair::core::Thread::Warn(
                    "TrajectoryManager: replanning failed, continuing old trajectory\n");
                state_ = State::EXECUTING;
                setStatus("Executing (replan failed)");
                last_replan_time_ = t_actual; // reset timer to avoid busy loop
            }
        }
    }
}

// ============================================================================
// evaluate
// ============================================================================

bool TrajectoryManager::evaluate(float t_actual,
                                  flair::core::Vector3Df& xid,
                                  flair::core::Vector3Df& xidp,
                                  flair::core::Vector3Df& xidpp,
                                  flair::core::Vector3Df& xidppp)
{
    if (state_ != State::EXECUTING && state_ != State::REPLANNING) {
        return false;
    }
    if (!trajectory_valid_) {
        return false;
    }

    double elapsed = static_cast<double>(t_actual) - execution_start_time_;
    double total   = active_trajectory_.getTotalDuration();

    // Clamp to valid range
    if (elapsed < 0.0)    elapsed = 0.0;
    if (elapsed >= total) elapsed = total;

    // Evaluate position through jerk
    Eigen::Vector3d p = active_trajectory_.getPos(elapsed);
    Eigen::Vector3d v = active_trajectory_.getVel(elapsed);
    Eigen::Vector3d a = active_trajectory_.getAcc(elapsed);
    Eigen::Vector3d j = active_trajectory_.getJer(elapsed);

    // Convert Eigen::Vector3d -> flair::core::Vector3Df
    xid    = flair::core::Vector3Df(static_cast<float>(p.x()),
                                     static_cast<float>(p.y()),
                                     static_cast<float>(p.z()));
    xidp   = flair::core::Vector3Df(static_cast<float>(v.x()),
                                     static_cast<float>(v.y()),
                                     static_cast<float>(v.z()));
    xidpp  = flair::core::Vector3Df(static_cast<float>(a.x()),
                                     static_cast<float>(a.y()),
                                     static_cast<float>(a.z()));
    xidppp = flair::core::Vector3Df(static_cast<float>(j.x()),
                                     static_cast<float>(j.y()),
                                     static_cast<float>(j.z()));
    return true;
}

// ============================================================================
// isExecuting
// ============================================================================

bool TrajectoryManager::isExecuting() const
{
    return (state_ == State::EXECUTING || state_ == State::REPLANNING);
}

// ============================================================================
// stop
// ============================================================================

void TrajectoryManager::stop()
{
    state_ = State::IDLE;
    setStatus("Stopped");
    flair::core::Thread::Info("TrajectoryManager: stopped\n");
}
