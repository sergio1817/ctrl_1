// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2024
//  filename:   TrajectoryManager.cpp
//
//  author:     Sergio Urzua
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  purpose:    IODevice-based trajectory planner with min-snap optimization
//
/*********************************************************************/

#include "TrajectoryManager.h"
#include <Matrix.h>
#include <MatrixDescriptor.h>
#include <IODevice.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <SpinBox.h>
#include <PushButton.h>
#include <Label.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <LayoutPosition.h>
#include <Layout.h>
#include <Thread.h>
#include <Vector3D.h>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <cstring>

using std::string;
using namespace flair::core;
using namespace flair::gui;


// ============================================================
// Constructor
// ============================================================
TrajectoryManager::TrajectoryManager(const LayoutPosition *position, string name)
    : IODevice(position->getLayout(), name),
      state_(State::IDLE),
      output_matrix_(NULL),
      last_pos_(0, 0, 0),
      last_vel_(0, 0, 0),
      last_acc_(0, 0, 0),
      last_jerk_(0, 0, 0),
      progress_(0.0f),
      num_segments_(0),
      total_duration_(0.0),
      trajectory_valid_(false),
      execution_start_time_(0.0),
      last_replan_time_(0.0),
      num_waypoints_(2),
      start_vel_(Eigen::Vector3d::Zero()),
      end_vel_(Eigen::Vector3d::Zero()),
      num_obstacles_(0)
{
    // --------------------------------------------------------
    // Output matrix with named elements (Flair pattern from Sliding_pos)
    // --------------------------------------------------------
    MatrixDescriptor *desc = new MatrixDescriptor(13, 1);
    desc->SetElementName(0, 0, "des_x");
    desc->SetElementName(1, 0, "des_y");
    desc->SetElementName(2, 0, "des_z");
    desc->SetElementName(3, 0, "des_vx");
    desc->SetElementName(4, 0, "des_vy");
    desc->SetElementName(5, 0, "des_vz");
    desc->SetElementName(6, 0, "des_ax");
    desc->SetElementName(7, 0, "des_ay");
    desc->SetElementName(8, 0, "des_az");
    desc->SetElementName(9, 0, "des_jx");
    desc->SetElementName(10, 0, "des_jy");
    desc->SetElementName(11, 0, "des_jz");
    desc->SetElementName(12, 0, "progress");
    output_matrix_ = new Matrix(this, desc, floatType, name);
    delete desc;

    // --------------------------------------------------------
    // GUI: Settings GroupBox
    // --------------------------------------------------------
    GroupBox *main_box = new GroupBox(position, name);
    settings_box_ = new GroupBox(main_box->NewRow(), "Planner Settings");

    max_vel_ = new DoubleSpinBox(settings_box_->NewRow(), "Max velocity", " m/s", 0.1, 5.0, 0.1, 2);
    max_acc_ = new DoubleSpinBox(settings_box_->LastRowLastCol(), "Max accel", " m/s2", 0.1, 10.0, 0.1, 2);
    safety_margin_ = new DoubleSpinBox(settings_box_->NewRow(), "Safety margin", " m", 0.0, 1.0, 0.05, 2);
    replan_period_ = new DoubleSpinBox(settings_box_->LastRowLastCol(), "Replan period", " s", 0.0, 10.0, 0.5, 1);

    // Waypoint count
    num_wp_spin_ = new SpinBox(settings_box_->NewRow(), "Num waypoints", 2, MAX_WAYPOINTS, 1);

    // Waypoint coordinates
    GroupBox *wp_box = new GroupBox(main_box->NewRow(), "Waypoints");
    for (int i = 0; i < MAX_WAYPOINTS; ++i) {
        char label_x[32], label_y[32], label_z[32];
        snprintf(label_x, sizeof(label_x), "WP%d x", i);
        snprintf(label_y, sizeof(label_y), "WP%d y", i);
        snprintf(label_z, sizeof(label_z), "WP%d z", i);
        wp_x_[i] = new DoubleSpinBox(wp_box->NewRow(), label_x, " m", -5.0, 5.0, 0.1, 2);
        wp_y_[i] = new DoubleSpinBox(wp_box->LastRowLastCol(), label_y, " m", -5.0, 5.0, 0.1, 2);
        wp_z_[i] = new DoubleSpinBox(wp_box->LastRowLastCol(), label_z, " m", -3.0, 0.0, 0.1, 2);
    }

    // Buttons
    GroupBox *ctrl_box = new GroupBox(main_box->NewRow(), "Control");
    plan_button_ = new PushButton(ctrl_box->NewRow(), "Plan trajectory");
    execute_button_ = new PushButton(ctrl_box->LastRowLastCol(), "Execute trajectory");
    stop_button_ = new PushButton(ctrl_box->LastRowLastCol(), "Stop trajectory");
    status_label_ = new Label(ctrl_box->NewRow(), "Status");
    status_label_->SetText("IDLE");

    // --------------------------------------------------------
    // DataPlots for trajectory visualization
    // (Use main_box->NewRow() for layout positions — the original
    //  'position' was consumed by the GroupBox constructor above)
    // --------------------------------------------------------
    // 2D XY trajectory plot
    DataPlot2D *xy_plot = new DataPlot2D(main_box->NewRow(), "XY Trajectory",
                                          "X [m]", -3, 3,
                                          "Y [m]", -3, 3);
    xy_plot->AddCurve(output_matrix_->Element(0, 0),
                      output_matrix_->Element(1, 0),
                      DataPlot::Red, "desired");

    // 1D position plots
    DataPlot1D *pos_x_plot = new DataPlot1D(main_box->NewRow(), "Desired X", -3, 3);
    pos_x_plot->AddCurve(output_matrix_->Element(0, 0), DataPlot::Red, "des_x");

    DataPlot1D *pos_y_plot = new DataPlot1D(main_box->LastRowLastCol(), "Desired Y", -3, 3);
    pos_y_plot->AddCurve(output_matrix_->Element(1, 0), DataPlot::Green, "des_y");

    DataPlot1D *pos_z_plot = new DataPlot1D(main_box->LastRowLastCol(), "Desired Z", -3, 0);
    pos_z_plot->AddCurve(output_matrix_->Element(2, 0), DataPlot::Blue, "des_z");

    // Velocity plot
    DataPlot1D *vel_plot = new DataPlot1D(main_box->NewRow(), "Desired Vel", -5, 5);
    vel_plot->AddCurve(output_matrix_->Element(3, 0), DataPlot::Red, "vx");
    vel_plot->AddCurve(output_matrix_->Element(4, 0), DataPlot::Green, "vy");
    vel_plot->AddCurve(output_matrix_->Element(5, 0), DataPlot::Blue, "vz");

    // Progress plot
    DataPlot1D *prog_plot = new DataPlot1D(main_box->NewRow(), "Progress", 0, 1.1f);
    prog_plot->AddCurve(output_matrix_->Element(12, 0), DataPlot::Black, "t/T");

    // --------------------------------------------------------
    // Initialize waypoints to defaults
    // --------------------------------------------------------
    for (int i = 0; i < MAX_WAYPOINTS; ++i) {
        waypoints_[i] = Eigen::Vector3d::Zero();
    }

    // Initialize segments
    for (int i = 0; i < MAX_SEGMENTS; ++i) {
        segments_[i].coeffs.setZero();
        segments_[i].duration = 0.0;
    }

    // Initialize obstacles
    for (int i = 0; i < MAX_OBSTACLES; ++i) {
        obstacles_[i].pos = Eigen::Vector3d::Zero();
        obstacles_[i].vel = Eigen::Vector3d::Zero();
        obstacles_[i].radius = 0.0;
    }

    // Add output to data log
    AddDataToLog(output_matrix_);
}

// ============================================================
// Destructor
// ============================================================
TrajectoryManager::~TrajectoryManager() {
    delete output_matrix_;
}

// ============================================================
// Update - called from control loop at each tick
// ============================================================
void TrajectoryManager::Update(Time time) {
    // Check GUI buttons
    if (plan_button_->Clicked()) {
        Vector3Df dummy_pos(0, 0, 0);
        Vector3Df dummy_vel(0, 0, 0);
        Plan(dummy_pos, dummy_vel);
    }
    if (execute_button_->Clicked() && state_ == State::PLANNED) {
        StartTraj();
        execution_start_time_ = static_cast<double>(time) / 1e9;
    }
    if (stop_button_->Clicked()) {
        StopTraj();
    }

    // Convert Flair time to seconds
    double t_sec = static_cast<double>(time) / 1e9;

    if (state_ == State::EXECUTING && trajectory_valid_) {
        double t_traj = t_sec - execution_start_time_;
        double elapsed = (t_traj < 0.0) ? 0.0 : ((t_traj > total_duration_) ? total_duration_ : t_traj);
        float prog = (total_duration_ > 1e-9) ? static_cast<float>(elapsed / total_duration_) : 1.0f;

        Eigen::Vector3d p = EvalPos(elapsed);
        Eigen::Vector3d v = EvalVel(elapsed);
        Eigen::Vector3d a = EvalAcc(elapsed);
        Eigen::Vector3d j = EvalJer(elapsed);

        // Thread-safe matrix update (Flair pattern)
        output_matrix_->GetMutex();
        output_matrix_->SetValueNoMutex(0, 0, static_cast<float>(p.x()));
        output_matrix_->SetValueNoMutex(1, 0, static_cast<float>(p.y()));
        output_matrix_->SetValueNoMutex(2, 0, static_cast<float>(p.z()));
        output_matrix_->SetValueNoMutex(3, 0, static_cast<float>(v.x()));
        output_matrix_->SetValueNoMutex(4, 0, static_cast<float>(v.y()));
        output_matrix_->SetValueNoMutex(5, 0, static_cast<float>(v.z()));
        output_matrix_->SetValueNoMutex(6, 0, static_cast<float>(a.x()));
        output_matrix_->SetValueNoMutex(7, 0, static_cast<float>(a.y()));
        output_matrix_->SetValueNoMutex(8, 0, static_cast<float>(a.z()));
        output_matrix_->SetValueNoMutex(9, 0, static_cast<float>(j.x()));
        output_matrix_->SetValueNoMutex(10, 0, static_cast<float>(j.y()));
        output_matrix_->SetValueNoMutex(11, 0, static_cast<float>(j.z()));
        output_matrix_->SetValueNoMutex(12, 0, prog);
        output_matrix_->ReleaseMutex();

        // Store for GetPosition/GetSpeed/etc accessors
        last_pos_ = Vector3Df(static_cast<float>(p.x()),
                              static_cast<float>(p.y()),
                              static_cast<float>(p.z()));
        last_vel_ = Vector3Df(static_cast<float>(v.x()),
                              static_cast<float>(v.y()),
                              static_cast<float>(v.z()));
        last_acc_ = Vector3Df(static_cast<float>(a.x()),
                              static_cast<float>(a.y()),
                              static_cast<float>(a.z()));
        last_jerk_ = Vector3Df(static_cast<float>(j.x()),
                               static_cast<float>(j.y()),
                               static_cast<float>(j.z()));
        progress_ = prog;

        // Signal data update to DataPlot framework
        output_matrix_->SetDataTime(time);
        ProcessUpdate(output_matrix_);

        // Check trajectory completion
        if (t_traj >= total_duration_) {
            state_ = State::IDLE;
            status_label_->SetText("IDLE (complete)");
            Info("trajectory complete\n");
        }

        // Check replan trigger
        double rp = replan_period_->Value();
        if (rp > 0.0 && (t_traj - last_replan_time_) >= rp) {
            last_replan_time_ = t_traj;
            // Could trigger Replan here if obstacle avoidance is active
        }
    } else {
        // Not executing - zero output
        output_matrix_->GetMutex();
        for (int i = 0; i < 13; ++i) {
            output_matrix_->SetValueNoMutex(i, 0, 0.0f);
        }
        output_matrix_->ReleaseMutex();

        last_pos_ = Vector3Df(0, 0, 0);
        last_vel_ = Vector3Df(0, 0, 0);
        last_acc_ = Vector3Df(0, 0, 0);
        last_jerk_ = Vector3Df(0, 0, 0);
        progress_ = 0.0f;

        output_matrix_->SetDataTime(time);
        ProcessUpdate(output_matrix_);
    }
}

// ============================================================
// Accessors
// ============================================================
void TrajectoryManager::GetPosition(Vector3Df &pos) const {
    pos = last_pos_;
}

void TrajectoryManager::GetSpeed(Vector3Df &vel) const {
    vel = last_vel_;
}

void TrajectoryManager::GetAcceleration(Vector3Df &acc) const {
    acc = last_acc_;
}

void TrajectoryManager::GetJerk(Vector3Df &jerk) const {
    jerk = last_jerk_;
}

Matrix *TrajectoryManager::GetMatrix() const {
    return output_matrix_;
}

float TrajectoryManager::GetProgress() const {
    return progress_;
}

bool TrajectoryManager::IsRunning() const {
    return state_ == State::EXECUTING;
}

// ============================================================
// Lifecycle
// ============================================================
void TrajectoryManager::StartTraj() {
    if (state_ == State::PLANNED && trajectory_valid_) {
        state_ = State::EXECUTING;
        last_replan_time_ = 0.0;
        status_label_->SetText("EXECUTING");
        Info("trajectory execution started\n");
    }
}

void TrajectoryManager::StopTraj() {
    state_ = State::IDLE;
    status_label_->SetText("IDLE (stopped)");
    Info("trajectory stopped\n");
}

// ============================================================
// Planning
// ============================================================
bool TrajectoryManager::Plan(const Vector3Df &current_pos,
                             const Vector3Df &current_vel) {
    state_ = State::PLANNING;
    status_label_->SetText("PLANNING...");

    ReadWaypointsFromGUI();

    start_vel_ = Eigen::Vector3d(current_vel.x, current_vel.y, current_vel.z);
    end_vel_ = Eigen::Vector3d::Zero();

    bool ok = SolveMinSnap();
    if (ok) {
        state_ = State::PLANNED;
        status_label_->SetText("PLANNED (ready)");
        Info("trajectory planned: %d segments, %.2f s\n", num_segments_, total_duration_);
    } else {
        state_ = State::IDLE;
        status_label_->SetText("PLAN FAILED");
        Warn("trajectory planning failed\n");
    }
    return ok;
}

bool TrajectoryManager::Replan(const Vector3Df &current_pos,
                               const Vector3Df &current_vel) {
    State prev = state_;
    state_ = State::REPLANNING;
    status_label_->SetText("REPLANNING...");

    // Update first waypoint to current position
    waypoints_[0] = Eigen::Vector3d(current_pos.x, current_pos.y, current_pos.z);
    start_vel_ = Eigen::Vector3d(current_vel.x, current_vel.y, current_vel.z);

    bool ok = SolveMinSnap();
    if (ok) {
        state_ = State::EXECUTING;
        status_label_->SetText("EXECUTING (replanned)");
    } else {
        state_ = prev;
        Warn("replanning failed, continuing with old trajectory\n");
    }
    return ok;
}

// ============================================================
// Obstacle management
// ============================================================
void TrajectoryManager::AddObstacle(const Vector3Df &pos, float radius) {
    if (num_obstacles_ < MAX_OBSTACLES) {
        obstacles_[num_obstacles_].pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
        obstacles_[num_obstacles_].vel = Eigen::Vector3d::Zero();
        obstacles_[num_obstacles_].radius = static_cast<double>(radius);
        num_obstacles_++;
    }
}

void TrajectoryManager::ClearObstacles() {
    num_obstacles_ = 0;
}

void TrajectoryManager::UpdateObstaclePosition(int idx, const Vector3Df &pos) {
    if (idx >= 0 && idx < num_obstacles_) {
        obstacles_[idx].pos = Eigen::Vector3d(pos.x, pos.y, pos.z);
    }
}

void TrajectoryManager::UpdateObstacleVelocity(int idx, const Vector3Df &vel) {
    if (idx >= 0 && idx < num_obstacles_) {
        obstacles_[idx].vel = Eigen::Vector3d(vel.x, vel.y, vel.z);
    }
}

// ============================================================
// Internal: Read waypoints from GUI spinboxes
// ============================================================
void TrajectoryManager::ReadWaypointsFromGUI() {
    num_waypoints_ = num_wp_spin_->Value();
    if (num_waypoints_ < 2) num_waypoints_ = 2;
    if (num_waypoints_ > MAX_WAYPOINTS) num_waypoints_ = MAX_WAYPOINTS;

    for (int i = 0; i < num_waypoints_; ++i) {
        waypoints_[i] = Eigen::Vector3d(wp_x_[i]->Value(),
                                         wp_y_[i]->Value(),
                                         wp_z_[i]->Value());
    }
}

// ============================================================
// Internal: Solve minimum-snap trajectory
// ============================================================
//
// Solves the closed-form unconstrained min-snap problem for M segments.
// Each segment is a degree-7 polynomial in t: p(t) = sum_{k=0}^{7} c_k * t^k
// Boundary conditions: position, velocity, acceleration, jerk at endpoints.
// Interior waypoint conditions: position continuity + C3 continuity.
//
// This is a standard banded linear system: 8M unknowns, 8M equations.
// For M <= 6, size is at most 48x48 - trivially fast.
//
bool TrajectoryManager::SolveMinSnap() {
    num_segments_ = num_waypoints_ - 1;
    if (num_segments_ < 1 || num_segments_ > MAX_SEGMENTS) {
        return false;
    }

    const int M = num_segments_;
    const int N = 8;  // coefficients per segment per axis
    const int dim = M * N;  // total unknowns per axis

    // Allocate time per segment based on distance / max_vel
    double v_max = max_vel_->Value();
    if (v_max < 0.1) v_max = 0.1;

    double durations[MAX_SEGMENTS];
    total_duration_ = 0.0;
    for (int i = 0; i < M; ++i) {
        double dist = (waypoints_[i + 1] - waypoints_[i]).norm();
        double t_seg = dist / v_max;
        if (t_seg < 0.5) t_seg = 0.5;  // minimum segment time
        durations[i] = t_seg;
        total_duration_ += t_seg;
    }

    // Solve for each axis independently
    for (int axis = 0; axis < 3; ++axis) {
        // Build and solve the linear system A * c = b
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dim, dim);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(dim);

        int row = 0;

        // Helper: powers of t and factorial coefficients for derivatives
        // p(t) = c0 + c1*t + c2*t^2 + ... + c7*t^7
        // p'(t) = c1 + 2*c2*t + 3*c3*t^2 + ... + 7*c7*t^6
        // p''(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + ... + 42*c7*t^5
        // p'''(t) = 6*c3 + 24*c4*t + 60*c5*t^2 + 120*c6*t^3 + 210*c7*t^4

        // Start boundary: p_0(0)=pos0, p_0'(0)=v0, p_0''(0)=0, p_0'''(0)=0
        {
            int seg_off = 0;
            // pos at t=0: c0 = pos
            A(row, seg_off + 0) = 1.0;
            b(row) = waypoints_[0](axis);
            row++;

            // vel at t=0: c1 = v0
            A(row, seg_off + 1) = 1.0;
            b(row) = start_vel_(axis);
            row++;

            // acc at t=0: 2*c2 = 0
            A(row, seg_off + 2) = 2.0;
            b(row) = 0.0;
            row++;

            // jerk at t=0: 6*c3 = 0
            A(row, seg_off + 3) = 6.0;
            b(row) = 0.0;
            row++;
        }

        // End boundary: p_{M-1}(T)=pos_end, p_{M-1}'(T)=v_end, p_{M-1}''(T)=0, p_{M-1}'''(T)=0
        {
            int seg_off = (M - 1) * N;
            double T = durations[M - 1];
            double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T, T6 = T5 * T, T7 = T6 * T;

            // pos at t=T
            double tp[8] = {1, T, T2, T3, T4, T5, T6, T7};
            for (int k = 0; k < N; ++k) A(row, seg_off + k) = tp[k];
            b(row) = waypoints_[M](axis);
            row++;

            // vel at t=T
            A(row, seg_off + 1) = 1.0;
            A(row, seg_off + 2) = 2.0 * T;
            A(row, seg_off + 3) = 3.0 * T2;
            A(row, seg_off + 4) = 4.0 * T3;
            A(row, seg_off + 5) = 5.0 * T4;
            A(row, seg_off + 6) = 6.0 * T5;
            A(row, seg_off + 7) = 7.0 * T6;
            b(row) = end_vel_(axis);
            row++;

            // acc at t=T
            A(row, seg_off + 2) = 2.0;
            A(row, seg_off + 3) = 6.0 * T;
            A(row, seg_off + 4) = 12.0 * T2;
            A(row, seg_off + 5) = 20.0 * T3;
            A(row, seg_off + 6) = 30.0 * T4;
            A(row, seg_off + 7) = 42.0 * T5;
            b(row) = 0.0;
            row++;

            // jerk at t=T
            A(row, seg_off + 3) = 6.0;
            A(row, seg_off + 4) = 24.0 * T;
            A(row, seg_off + 5) = 60.0 * T2;
            A(row, seg_off + 6) = 120.0 * T3;
            A(row, seg_off + 7) = 210.0 * T4;
            b(row) = 0.0;
            row++;
        }

        // Interior waypoint and continuity conditions
        for (int i = 0; i < M - 1; ++i) {
            int seg_off_i = i * N;
            int seg_off_j = (i + 1) * N;
            double T = durations[i];
            double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T, T6 = T5 * T, T7 = T6 * T;
            double tp[8] = {1, T, T2, T3, T4, T5, T6, T7};

            // Position at end of segment i = waypoint[i+1]
            for (int k = 0; k < N; ++k) A(row, seg_off_i + k) = tp[k];
            b(row) = waypoints_[i + 1](axis);
            row++;

            // Position at start of segment i+1 = waypoint[i+1]
            A(row, seg_off_j + 0) = 1.0;
            b(row) = waypoints_[i + 1](axis);
            row++;

            // Velocity continuity: p_i'(T) = p_{i+1}'(0)
            A(row, seg_off_i + 1) = 1.0;
            A(row, seg_off_i + 2) = 2.0 * T;
            A(row, seg_off_i + 3) = 3.0 * T2;
            A(row, seg_off_i + 4) = 4.0 * T3;
            A(row, seg_off_i + 5) = 5.0 * T4;
            A(row, seg_off_i + 6) = 6.0 * T5;
            A(row, seg_off_i + 7) = 7.0 * T6;
            A(row, seg_off_j + 1) = -1.0;
            b(row) = 0.0;
            row++;

            // Acceleration continuity: p_i''(T) = p_{i+1}''(0)
            A(row, seg_off_i + 2) = 2.0;
            A(row, seg_off_i + 3) = 6.0 * T;
            A(row, seg_off_i + 4) = 12.0 * T2;
            A(row, seg_off_i + 5) = 20.0 * T3;
            A(row, seg_off_i + 6) = 30.0 * T4;
            A(row, seg_off_i + 7) = 42.0 * T5;
            A(row, seg_off_j + 2) = -2.0;
            b(row) = 0.0;
            row++;

            // Jerk continuity: p_i'''(T) = p_{i+1}'''(0)
            A(row, seg_off_i + 3) = 6.0;
            A(row, seg_off_i + 4) = 24.0 * T;
            A(row, seg_off_i + 5) = 60.0 * T2;
            A(row, seg_off_i + 6) = 120.0 * T3;
            A(row, seg_off_i + 7) = 210.0 * T4;
            A(row, seg_off_j + 3) = -6.0;
            b(row) = 0.0;
            row++;

            // Snap continuity: p_i''''(T) = p_{i+1}''''(0)
            A(row, seg_off_i + 4) = 24.0;
            A(row, seg_off_i + 5) = 120.0 * T;
            A(row, seg_off_i + 6) = 360.0 * T2;
            A(row, seg_off_i + 7) = 840.0 * T3;
            A(row, seg_off_j + 4) = -24.0;
            b(row) = 0.0;
            row++;

            // Crackle continuity (5th derivative): p_i^(5)(T) = p_{i+1}^(5)(0)
            A(row, seg_off_i + 5) = 120.0;
            A(row, seg_off_i + 6) = 720.0 * T;
            A(row, seg_off_i + 7) = 2520.0 * T2;
            A(row, seg_off_j + 5) = -120.0;
            b(row) = 0.0;
            row++;

            // 6th derivative continuity
            A(row, seg_off_i + 6) = 720.0;
            A(row, seg_off_i + 7) = 5040.0 * T;
            A(row, seg_off_j + 6) = -720.0;
            b(row) = 0.0;
            row++;
        }

        // Solve with Eigen (ColPivHouseholderQR for robustness)
        Eigen::VectorXd c = A.colPivHouseholderQr().solve(b);

        // Check solution quality
        double residual = (A * c - b).norm();
        if (residual > 1e-6) {
            Warn("min-snap solve: large residual %.6e for axis %d\n", residual, axis);
            return false;
        }

        // Extract coefficients into segment storage
        for (int i = 0; i < M; ++i) {
            for (int k = 0; k < N; ++k) {
                segments_[i].coeffs(axis, k) = c(i * N + k);
            }
            segments_[i].duration = durations[i];
        }
    }

    trajectory_valid_ = true;
    return true;
}

// ============================================================
// Internal: Locate which segment a time t falls into
// ============================================================
int TrajectoryManager::LocateSegment(double t, double &t_local) const {
    double accumulated = 0.0;
    for (int i = 0; i < num_segments_; ++i) {
        if (t <= accumulated + segments_[i].duration || i == num_segments_ - 1) {
            t_local = t - accumulated;
            if (t_local < 0.0) t_local = 0.0;
            if (t_local > segments_[i].duration) t_local = segments_[i].duration;
            return i;
        }
        accumulated += segments_[i].duration;
    }
    t_local = segments_[num_segments_ - 1].duration;
    return num_segments_ - 1;
}

// ============================================================
// Internal: Evaluate trajectory at time t
// ============================================================
// p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5 + c6*t^6 + c7*t^7
// Using Horner's method for efficiency:
// p(t) = c0 + t*(c1 + t*(c2 + t*(c3 + t*(c4 + t*(c5 + t*(c6 + t*c7))))))

Eigen::Vector3d TrajectoryManager::EvalPos(double t) const {
    if (!trajectory_valid_ || num_segments_ < 1) return Eigen::Vector3d::Zero();

    double t_local;
    int seg = LocateSegment(t, t_local);
    const Eigen::Matrix<double, 3, 8> &c = segments_[seg].coeffs;

    // Horner evaluation
    Eigen::Vector3d result = c.col(7);
    for (int k = 6; k >= 0; --k) {
        result = result * t_local + c.col(k);
    }
    return result;
}

Eigen::Vector3d TrajectoryManager::EvalVel(double t) const {
    if (!trajectory_valid_ || num_segments_ < 1) return Eigen::Vector3d::Zero();

    double t_local;
    int seg = LocateSegment(t, t_local);
    const Eigen::Matrix<double, 3, 8> &c = segments_[seg].coeffs;

    // p'(t) = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4 + 6*c6*t^5 + 7*c7*t^6
    Eigen::Vector3d result = 7.0 * c.col(7);
    result = result * t_local + 6.0 * c.col(6);
    result = result * t_local + 5.0 * c.col(5);
    result = result * t_local + 4.0 * c.col(4);
    result = result * t_local + 3.0 * c.col(3);
    result = result * t_local + 2.0 * c.col(2);
    result = result * t_local + 1.0 * c.col(1);
    return result;
}

Eigen::Vector3d TrajectoryManager::EvalAcc(double t) const {
    if (!trajectory_valid_ || num_segments_ < 1) return Eigen::Vector3d::Zero();

    double t_local;
    int seg = LocateSegment(t, t_local);
    const Eigen::Matrix<double, 3, 8> &c = segments_[seg].coeffs;

    // p''(t) = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3 + 30*c6*t^4 + 42*c7*t^5
    Eigen::Vector3d result = 42.0 * c.col(7);
    result = result * t_local + 30.0 * c.col(6);
    result = result * t_local + 20.0 * c.col(5);
    result = result * t_local + 12.0 * c.col(4);
    result = result * t_local + 6.0 * c.col(3);
    result = result * t_local + 2.0 * c.col(2);
    return result;
}

Eigen::Vector3d TrajectoryManager::EvalJer(double t) const {
    if (!trajectory_valid_ || num_segments_ < 1) return Eigen::Vector3d::Zero();

    double t_local;
    int seg = LocateSegment(t, t_local);
    const Eigen::Matrix<double, 3, 8> &c = segments_[seg].coeffs;

    // p'''(t) = 6*c3 + 24*c4*t + 60*c5*t^2 + 120*c6*t^3 + 210*c7*t^4
    Eigen::Vector3d result = 210.0 * c.col(7);
    result = result * t_local + 120.0 * c.col(6);
    result = result * t_local + 60.0 * c.col(5);
    result = result * t_local + 24.0 * c.col(4);
    result = result * t_local + 6.0 * c.col(3);
    return result;
}
