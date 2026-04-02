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
//              and obstacle avoidance (occupancy grid + A* + SFC + corridor QP)
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
#include <ComboBox.h>
#include <Label.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <LayoutPosition.h>
#include <Layout.h>
#include <Tab.h>
#include <TabWidget.h>
#include <Thread.h>
#include <Vector3D.h>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <queue>
#include <unordered_map>
#include <limits>

using std::string;
using namespace flair::core;
using namespace flair::gui;

// Workspace bounds (NED: z negative = up, ground at z=0)
// Default workspace bounds (overridden by GUI values in Plan())

// ============================================================
// Constructor
// ============================================================
TrajectoryManager::TrajectoryManager(const LayoutPosition *position,
                                     TabWidget *plots_tab,
                                     string name)
    : IODevice(position->getLayout(), name),
      state_(State::IDLE),
      output_matrix_(NULL),
      last_pos_(0, 0, 0),
      WS_X_MIN(-5.0), WS_X_MAX(5.0),
      WS_Y_MIN(-5.0), WS_Y_MAX(5.0),
      WS_Z_MIN(-3.0), WS_Z_MAX(0.0),
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
      num_obstacles_(0),
      grid_nx_(0), grid_ny_(0), grid_nz_(0),
      grid_res_(0.1),
      grid_origin_(WS_X_MIN, WS_Y_MIN, WS_Z_MIN)
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

    // Obstacle avoidance settings
    GroupBox *obs_box = new GroupBox(main_box->NewRow(), "Obstacle Avoidance");
    obstacle_avoidance_mode_ = new ComboBox(obs_box->NewRow(), "Obstacle avoidance");
    obstacle_avoidance_mode_->AddItem("Disabled");
    obstacle_avoidance_mode_->AddItem("Enabled");
    obstacle_radius_spin_ = new DoubleSpinBox(obs_box->LastRowLastCol(), "Obstacle radius", " m", 0.05, 1.0, 0.05, 2);

    // Grid / workspace settings
    GroupBox *grid_box = new GroupBox(main_box->NewRow(), "Grid Settings");
    grid_res_spin_ = new DoubleSpinBox(grid_box->NewRow(), "Grid resolution", " m", 0.05, 0.5, 0.05, 2);
    ws_xy_range_ = new DoubleSpinBox(grid_box->LastRowLastCol(), "XY range", " m", 1.0, 10.0, 0.5, 1);
    ws_z_max_alt_ = new DoubleSpinBox(grid_box->LastRowLastCol(), "Max altitude", " m", 0.5, 5.0, 0.5, 1);

    // Waypoint count
    num_wp_spin_ = new SpinBox(settings_box_->NewRow(), "Num waypoints", 2, MAX_GUI_WAYPOINTS, 1);

    // Waypoint coordinates
    GroupBox *wp_box = new GroupBox(main_box->NewRow(), "Waypoints");
    for (int i = 0; i < MAX_GUI_WAYPOINTS; ++i) {
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
    // DataPlots on a dedicated tab under the Position TabWidget
    // --------------------------------------------------------
    Tab *plot_tab = new Tab(plots_tab, "Plots Trajectory");

    // 2D XY trajectory plot
    DataPlot2D *xy_plot = new DataPlot2D(plot_tab->NewRow(), "XY Trajectory",
                                          "X [m]", -3, 3,
                                          "Y [m]", -3, 3);
    xy_plot->AddCurve(output_matrix_->Element(0, 0),
                      output_matrix_->Element(1, 0),
                      DataPlot::Red, "desired");


    // Velocity plot
    DataPlot1D *vel_plot = new DataPlot1D(plot_tab->LastRowLastCol(), "Desired Vel", -5, 5);
    vel_plot->AddCurve(output_matrix_->Element(3, 0), DataPlot::Red, "vx");
    vel_plot->AddCurve(output_matrix_->Element(4, 0), DataPlot::Green, "vy");
    vel_plot->AddCurve(output_matrix_->Element(5, 0), DataPlot::Blue, "vz");

    // Progress plot
    DataPlot1D *prog_plot = new DataPlot1D(plot_tab->LastRowLastCol(), "Progress", 0, 1.1f);
    prog_plot->AddCurve(output_matrix_->Element(12, 0), DataPlot::Black, "t/T");

    // 1D position plots
    DataPlot1D *pos_x_plot = new DataPlot1D(plot_tab->NewRow(), "Desired X", -3, 3);
    pos_x_plot->AddCurve(output_matrix_->Element(0, 0), DataPlot::Red, "des_x");

    DataPlot1D *pos_y_plot = new DataPlot1D(plot_tab->LastRowLastCol(), "Desired Y", -3, 3);
    pos_y_plot->AddCurve(output_matrix_->Element(1, 0), DataPlot::Green, "des_y");

    DataPlot1D *pos_z_plot = new DataPlot1D(plot_tab->LastRowLastCol(), "Desired Z", -3, 0);
    pos_z_plot->AddCurve(output_matrix_->Element(2, 0), DataPlot::Blue, "des_z");

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

    // Pre-allocate occupancy grid (0.1m resolution)
    InitGrid(0.1);

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

        // Check trajectory completion — hold final position
        if (t_traj >= total_duration_) {
            // Store the final position (evaluated at t = total_duration)
            Eigen::Vector3d p_end = EvalPos(total_duration_);
            last_pos_ = Vector3Df(static_cast<float>(p_end.x()),
                                  static_cast<float>(p_end.y()),
                                  static_cast<float>(p_end.z()));
            last_vel_ = Vector3Df(0, 0, 0);
            last_acc_ = Vector3Df(0, 0, 0);
            last_jerk_ = Vector3Df(0, 0, 0);
            progress_ = 1.0f;
            state_ = State::HOLDING;
            status_label_->SetText("Holding position");
            Info("trajectory complete, holding final position\n");
        }

        // Check replan trigger
        double rp = replan_period_->Value();
        if (rp > 0.0 && (t_traj - last_replan_time_) >= rp) {
            last_replan_time_ = t_traj;
        }
    } else if (state_ == State::HOLDING) {
        // Holding final position — output last_pos_ with zero derivatives
        output_matrix_->GetMutex();
        output_matrix_->SetValueNoMutex(0, 0, last_pos_.x);
        output_matrix_->SetValueNoMutex(1, 0, last_pos_.y);
        output_matrix_->SetValueNoMutex(2, 0, last_pos_.z);
        for (int i = 3; i < 12; ++i) {
            output_matrix_->SetValueNoMutex(i, 0, 0.0f); // vel, acc, jerk = 0
        }
        output_matrix_->SetValueNoMutex(12, 0, 1.0f); // progress = 100%
        output_matrix_->ReleaseMutex();

        output_matrix_->SetDataTime(time);
        ProcessUpdate(output_matrix_);
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
    return state_ == State::EXECUTING || state_ == State::HOLDING;
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

    // Validate Z (NED): warn if any waypoint z > -0.3 (too close to ground)
    for (int i = 0; i < num_waypoints_; ++i) {
        if (waypoints_[i].z() > -0.3) {
            Warn("WP%d z=%.2f is close to ground (z=0 in NED). "
                 "Flight altitude should be z < -0.3\n", i, waypoints_[i].z());
        }
    }

    // Update obstacle radius from GUI
    double obs_r = obstacle_radius_spin_->Value();
    for (int i = 0; i < num_obstacles_; ++i) {
        obstacles_[i].radius = obs_r;
    }

    // Reinitialize grid with current GUI resolution and workspace
    double res = grid_res_spin_->Value();
    double xy_range = ws_xy_range_->Value();
    double z_alt = ws_z_max_alt_->Value();
    WS_X_MIN = -xy_range;  WS_X_MAX = xy_range;
    WS_Y_MIN = -xy_range;  WS_Y_MAX = xy_range;
    WS_Z_MIN = -z_alt;     WS_Z_MAX = 0.0;
    InitGrid(res);

    // Use full obstacle avoidance pipeline if enabled and obstacles present
    bool ok;
    if (obstacle_avoidance_mode_->CurrentIndex() == 1 && num_obstacles_ > 0) {
        ok = PlanWithObstacleAvoidance();
    } else {
        ok = SolveMinSnap();
    }

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

    // Update obstacle radius from GUI
    double obs_r = obstacle_radius_spin_->Value();
    for (int i = 0; i < num_obstacles_; ++i) {
        obstacles_[i].radius = obs_r;
    }

    bool ok;
    if (obstacle_avoidance_mode_->CurrentIndex() == 1 && num_obstacles_ > 0) {
        ok = PlanWithObstacleAvoidance();
    } else {
        ok = SolveMinSnap();
    }

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
    if (num_waypoints_ > MAX_GUI_WAYPOINTS) num_waypoints_ = MAX_GUI_WAYPOINTS;

    for (int i = 0; i < num_waypoints_; ++i) {
        waypoints_[i] = Eigen::Vector3d(wp_x_[i]->Value(),
                                         wp_y_[i]->Value(),
                                         wp_z_[i]->Value());
    }
}

// ============================================================
// Internal: Solve minimum-snap trajectory (unconstrained)
// ============================================================
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

        // Start boundary: p_0(0)=pos0, p_0'(0)=v0, p_0''(0)=0, p_0'''(0)=0
        {
            int seg_off = 0;
            A(row, seg_off + 0) = 1.0;
            b(row) = waypoints_[0](axis);
            row++;

            A(row, seg_off + 1) = 1.0;
            b(row) = start_vel_(axis);
            row++;

            A(row, seg_off + 2) = 2.0;
            b(row) = 0.0;
            row++;

            A(row, seg_off + 3) = 6.0;
            b(row) = 0.0;
            row++;
        }

        // End boundary: p_{M-1}(T)=pos_end, p_{M-1}'(T)=v_end, p_{M-1}''(T)=0, p_{M-1}'''(T)=0
        {
            int seg_off = (M - 1) * N;
            double T = durations[M - 1];
            double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T, T6 = T5 * T, T7 = T6 * T;

            double tp[8] = {1, T, T2, T3, T4, T5, T6, T7};
            for (int k = 0; k < N; ++k) A(row, seg_off + k) = tp[k];
            b(row) = waypoints_[M](axis);
            row++;

            A(row, seg_off + 1) = 1.0;
            A(row, seg_off + 2) = 2.0 * T;
            A(row, seg_off + 3) = 3.0 * T2;
            A(row, seg_off + 4) = 4.0 * T3;
            A(row, seg_off + 5) = 5.0 * T4;
            A(row, seg_off + 6) = 6.0 * T5;
            A(row, seg_off + 7) = 7.0 * T6;
            b(row) = end_vel_(axis);
            row++;

            A(row, seg_off + 2) = 2.0;
            A(row, seg_off + 3) = 6.0 * T;
            A(row, seg_off + 4) = 12.0 * T2;
            A(row, seg_off + 5) = 20.0 * T3;
            A(row, seg_off + 6) = 30.0 * T4;
            A(row, seg_off + 7) = 42.0 * T5;
            b(row) = 0.0;
            row++;

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

            // Velocity continuity
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

            // Acceleration continuity
            A(row, seg_off_i + 2) = 2.0;
            A(row, seg_off_i + 3) = 6.0 * T;
            A(row, seg_off_i + 4) = 12.0 * T2;
            A(row, seg_off_i + 5) = 20.0 * T3;
            A(row, seg_off_i + 6) = 30.0 * T4;
            A(row, seg_off_i + 7) = 42.0 * T5;
            A(row, seg_off_j + 2) = -2.0;
            b(row) = 0.0;
            row++;

            // Jerk continuity
            A(row, seg_off_i + 3) = 6.0;
            A(row, seg_off_i + 4) = 24.0 * T;
            A(row, seg_off_i + 5) = 60.0 * T2;
            A(row, seg_off_i + 6) = 120.0 * T3;
            A(row, seg_off_i + 7) = 210.0 * T4;
            A(row, seg_off_j + 3) = -6.0;
            b(row) = 0.0;
            row++;

            // Snap continuity
            A(row, seg_off_i + 4) = 24.0;
            A(row, seg_off_i + 5) = 120.0 * T;
            A(row, seg_off_i + 6) = 360.0 * T2;
            A(row, seg_off_i + 7) = 840.0 * T3;
            A(row, seg_off_j + 4) = -24.0;
            b(row) = 0.0;
            row++;

            // Crackle continuity (5th derivative)
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
// Internal: Evaluate trajectory at time t (Horner's method)
// ============================================================
Eigen::Vector3d TrajectoryManager::EvalPos(double t) const {
    if (!trajectory_valid_ || num_segments_ < 1) return Eigen::Vector3d::Zero();

    double t_local;
    int seg = LocateSegment(t, t_local);
    const Eigen::Matrix<double, 3, 8> &c = segments_[seg].coeffs;

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

    Eigen::Vector3d result = 210.0 * c.col(7);
    result = result * t_local + 120.0 * c.col(6);
    result = result * t_local + 60.0 * c.col(5);
    result = result * t_local + 24.0 * c.col(4);
    result = result * t_local + 6.0 * c.col(3);
    return result;
}


// ################################################################
// ################################################################
//  OBSTACLE AVOIDANCE PIPELINE (inline grid + A* + SFC + corridor)
// ################################################################
// ################################################################

// ============================================================
// Occupancy Grid (inline implementation)
// ============================================================

void TrajectoryManager::InitGrid(double res) {
    grid_res_ = res;
    grid_origin_ = Eigen::Vector3d(WS_X_MIN, WS_Y_MIN, WS_Z_MIN);
    grid_nx_ = static_cast<int>(std::ceil((WS_X_MAX - WS_X_MIN) / res));
    grid_ny_ = static_cast<int>(std::ceil((WS_Y_MAX - WS_Y_MIN) / res));
    grid_nz_ = static_cast<int>(std::ceil((WS_Z_MAX - WS_Z_MIN) / res));
    grid_data_.assign(static_cast<size_t>(grid_nx_) * grid_ny_ * grid_nz_, 0u);
}

void TrajectoryManager::ClearGrid() {
    std::fill(grid_data_.begin(), grid_data_.end(), 0u);
}

bool TrajectoryManager::GridInBounds(int ix, int iy, int iz) const {
    return ix >= 0 && ix < grid_nx_ &&
           iy >= 0 && iy < grid_ny_ &&
           iz >= 0 && iz < grid_nz_;
}

bool TrajectoryManager::IsOccupied(int ix, int iy, int iz) const {
    if (!GridInBounds(ix, iy, iz)) return true;  // out of bounds = occupied
    size_t idx = static_cast<size_t>(ix) * grid_ny_ * grid_nz_
               + static_cast<size_t>(iy) * grid_nz_
               + static_cast<size_t>(iz);
    return grid_data_[idx] != 0u;
}

bool TrajectoryManager::IsOccupiedWorld(const Eigen::Vector3d &pos) const {
    Eigen::Vector3i gi = WorldToGrid(pos);
    return IsOccupied(gi.x(), gi.y(), gi.z());
}

Eigen::Vector3i TrajectoryManager::WorldToGrid(const Eigen::Vector3d &pos) const {
    int ix = static_cast<int>(std::floor((pos.x() - grid_origin_.x()) / grid_res_));
    int iy = static_cast<int>(std::floor((pos.y() - grid_origin_.y()) / grid_res_));
    int iz = static_cast<int>(std::floor((pos.z() - grid_origin_.z()) / grid_res_));
    return Eigen::Vector3i(ix, iy, iz);
}

Eigen::Vector3d TrajectoryManager::GridToWorld(int ix, int iy, int iz) const {
    return Eigen::Vector3d(
        grid_origin_.x() + (ix + 0.5) * grid_res_,
        grid_origin_.y() + (iy + 0.5) * grid_res_,
        grid_origin_.z() + (iz + 0.5) * grid_res_);
}

void TrajectoryManager::MarkSphereOccupied(const Eigen::Vector3d &center, double radius) {
    Eigen::Vector3i lo = WorldToGrid(center - Eigen::Vector3d(radius, radius, radius));
    Eigen::Vector3i hi = WorldToGrid(center + Eigen::Vector3d(radius, radius, radius));

    int ix0 = std::max(lo.x(), 0);
    int iy0 = std::max(lo.y(), 0);
    int iz0 = std::max(lo.z(), 0);
    int ix1 = std::min(hi.x(), grid_nx_ - 1);
    int iy1 = std::min(hi.y(), grid_ny_ - 1);
    int iz1 = std::min(hi.z(), grid_nz_ - 1);

    double r2 = radius * radius;
    for (int ix = ix0; ix <= ix1; ++ix) {
        for (int iy = iy0; iy <= iy1; ++iy) {
            for (int iz = iz0; iz <= iz1; ++iz) {
                Eigen::Vector3d vc = GridToWorld(ix, iy, iz);
                double dx = vc.x() - center.x();
                double dy = vc.y() - center.y();
                double dz = vc.z() - center.z();
                if (dx*dx + dy*dy + dz*dz <= r2) {
                    size_t idx = static_cast<size_t>(ix) * grid_ny_ * grid_nz_
                               + static_cast<size_t>(iy) * grid_nz_
                               + static_cast<size_t>(iz);
                    grid_data_[idx] = 1u;
                }
            }
        }
    }
}

bool TrajectoryManager::IsSegmentFree(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const {
    Eigen::Vector3d diff = b - a;
    double length = diff.norm();
    if (length < 1e-9) return !IsOccupiedWorld(a);

    int steps = static_cast<int>(std::ceil(length / (grid_res_ * 0.5)));
    Eigen::Vector3d step = diff / static_cast<double>(steps);
    for (int i = 0; i <= steps; ++i) {
        Eigen::Vector3d pt = a + step * static_cast<double>(i);
        if (IsOccupiedWorld(pt)) return false;
    }
    return true;
}

// ============================================================
// Build occupancy grid from obstacles + ground plane
// ============================================================
void TrajectoryManager::BuildOccupancyGrid() {
    ClearGrid();

    double sm = safety_margin_->Value();

    // Mark ground plane: z >= -safety_margin is occupied (NED: z=0 is ground)
    // In grid coords, this means voxels whose world z >= -sm
    for (int ix = 0; ix < grid_nx_; ++ix) {
        for (int iy = 0; iy < grid_ny_; ++iy) {
            for (int iz = 0; iz < grid_nz_; ++iz) {
                Eigen::Vector3d wc = GridToWorld(ix, iy, iz);
                if (wc.z() >= -sm) {
                    size_t idx = static_cast<size_t>(ix) * grid_ny_ * grid_nz_
                               + static_cast<size_t>(iy) * grid_nz_
                               + static_cast<size_t>(iz);
                    grid_data_[idx] = 1u;
                }
            }
        }
    }

    // Mark each obstacle as inflated sphere (radius + safety_margin)
    for (int i = 0; i < num_obstacles_; ++i) {
        MarkSphereOccupied(obstacles_[i].pos, obstacles_[i].radius + sm);
    }
}

// ============================================================
// A* path search with 26-connectivity
// ============================================================
bool TrajectoryManager::FindPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                                  std::vector<Eigen::Vector3d> &path) {
    path.clear();

    Eigen::Vector3i sg = WorldToGrid(start);
    Eigen::Vector3i gg = WorldToGrid(goal);
    int sx = sg.x(), sy = sg.y(), sz = sg.z();
    int gx = gg.x(), gy = gg.y(), gz = gg.z();

    if (!GridInBounds(sx, sy, sz) || !GridInBounds(gx, gy, gz)) return false;
    if (IsOccupied(sx, sy, sz) || IsOccupied(gx, gy, gz)) return false;

    // Trivial case
    if (sx == gx && sy == gy && sz == gz) {
        path.push_back(start);
        path.push_back(goal);
        return true;
    }

    // A* data structures
    struct Node {
        int x, y, z;
        double g, f;
        int parent;
    };

    struct Compare {
        bool operator()(const std::pair<double, int> &a,
                        const std::pair<double, int> &b) const {
            return a.first > b.first;
        }
    };

    std::vector<Node> nodes;
    nodes.reserve(4096);
    std::priority_queue<std::pair<double, int>,
                        std::vector<std::pair<double, int> >,
                        Compare> open_q;
    std::unordered_map<int64_t, int> closed;
    std::unordered_map<int64_t, int> open_set;

    int64_t ny64 = static_cast<int64_t>(grid_ny_);
    int64_t nz64 = static_cast<int64_t>(grid_nz_);

    // Lambda-like helper for encoding (C++11 compatible)
    #define ENCODE_IDX(x_, y_, z_) (static_cast<int64_t>(x_) * ny64 * nz64 + static_cast<int64_t>(y_) * nz64 + static_cast<int64_t>(z_))

    auto eucDist = [](int x1, int y1, int z1, int x2, int y2, int z2) -> double {
        double dx = static_cast<double>(x2 - x1);
        double dy = static_cast<double>(y2 - y1);
        double dz = static_cast<double>(z2 - z1);
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    Node start_n;
    start_n.x = sx; start_n.y = sy; start_n.z = sz;
    start_n.g = 0.0;
    start_n.f = eucDist(sx, sy, sz, gx, gy, gz) * grid_res_;
    start_n.parent = -1;
    nodes.push_back(start_n);
    open_set[ENCODE_IDX(sx, sy, sz)] = 0;
    open_q.push(std::make_pair(start_n.f, 0));

    // 26-connectivity offsets and costs
    static const int offsets[26][3] = {
        { 1, 0, 0}, {-1, 0, 0}, { 0, 1, 0}, { 0,-1, 0}, { 0, 0, 1}, { 0, 0,-1},
        { 1, 1, 0}, { 1,-1, 0}, {-1, 1, 0}, {-1,-1, 0},
        { 1, 0, 1}, { 1, 0,-1}, {-1, 0, 1}, {-1, 0,-1},
        { 0, 1, 1}, { 0, 1,-1}, { 0,-1, 1}, { 0,-1,-1},
        { 1, 1, 1}, { 1, 1,-1}, { 1,-1, 1}, { 1,-1,-1},
        {-1, 1, 1}, {-1, 1,-1}, {-1,-1, 1}, {-1,-1,-1}
    };
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
    const int MAX_ITER = grid_nx_ * grid_ny_ * grid_nz_;
    int iter = 0;

    while (!open_q.empty() && iter < MAX_ITER) {
        ++iter;
        std::pair<double, int> top = open_q.top();
        open_q.pop();
        int cur_idx = top.second;
        const Node &cur = nodes[cur_idx];
        int64_t cur_key = ENCODE_IDX(cur.x, cur.y, cur.z);

        if (closed.count(cur_key)) continue;
        closed[cur_key] = cur_idx;

        if (cur.x == gx && cur.y == gy && cur.z == gz) {
            found = true;
            goal_node_idx = cur_idx;
            break;
        }

        for (int ni = 0; ni < 26; ++ni) {
            int nx_i = cur.x + offsets[ni][0];
            int ny_i = cur.y + offsets[ni][1];
            int nz_i = cur.z + offsets[ni][2];

            if (!GridInBounds(nx_i, ny_i, nz_i)) continue;
            if (IsOccupied(nx_i, ny_i, nz_i)) continue;

            // Check diagonal safety (no corner cutting)
            int dx = offsets[ni][0], dy = offsets[ni][1], dz = offsets[ni][2];
            int nz_count = (dx != 0 ? 1 : 0) + (dy != 0 ? 1 : 0) + (dz != 0 ? 1 : 0);
            bool legal = true;
            if (nz_count == 2) {
                if (dx != 0 && dy != 0) {
                    if (IsOccupied(cur.x + dx, cur.y, cur.z) ||
                        IsOccupied(cur.x, cur.y + dy, cur.z)) legal = false;
                } else if (dx != 0 && dz != 0) {
                    if (IsOccupied(cur.x + dx, cur.y, cur.z) ||
                        IsOccupied(cur.x, cur.y, cur.z + dz)) legal = false;
                } else {
                    if (IsOccupied(cur.x, cur.y + dy, cur.z) ||
                        IsOccupied(cur.x, cur.y, cur.z + dz)) legal = false;
                }
            } else if (nz_count == 3) {
                if (IsOccupied(cur.x + dx, cur.y, cur.z) ||
                    IsOccupied(cur.x, cur.y + dy, cur.z) ||
                    IsOccupied(cur.x, cur.y, cur.z + dz) ||
                    IsOccupied(cur.x + dx, cur.y + dy, cur.z) ||
                    IsOccupied(cur.x + dx, cur.y, cur.z + dz) ||
                    IsOccupied(cur.x, cur.y + dy, cur.z + dz)) legal = false;
            }
            if (!legal) continue;

            int64_t nb_key = ENCODE_IDX(nx_i, ny_i, nz_i);
            if (closed.count(nb_key)) continue;

            double new_g = cur.g + step_costs[ni] * grid_res_;
            double h = eucDist(nx_i, ny_i, nz_i, gx, gy, gz) * grid_res_;
            double new_f = new_g + h;

            std::unordered_map<int64_t, int>::iterator oit = open_set.find(nb_key);
            if (oit != open_set.end()) {
                Node &existing = nodes[oit->second];
                if (new_g < existing.g) {
                    existing.g = new_g;
                    existing.f = new_f;
                    existing.parent = cur_idx;
                    open_q.push(std::make_pair(new_f, oit->second));
                }
            } else {
                Node nb;
                nb.x = nx_i; nb.y = ny_i; nb.z = nz_i;
                nb.g = new_g; nb.f = new_f;
                nb.parent = cur_idx;
                int nb_si = static_cast<int>(nodes.size());
                nodes.push_back(nb);
                open_set[nb_key] = nb_si;
                open_q.push(std::make_pair(new_f, nb_si));
            }
        }
    }

    #undef ENCODE_IDX

    if (!found) return false;

    // Reconstruct path
    std::vector<Eigen::Vector3d> raw_path;
    int idx = goal_node_idx;
    while (idx >= 0) {
        const Node &n = nodes[idx];
        raw_path.push_back(GridToWorld(n.x, n.y, n.z));
        idx = n.parent;
    }
    std::reverse(raw_path.begin(), raw_path.end());

    // Use exact start/goal positions
    if (!raw_path.empty()) raw_path.front() = start;
    if (raw_path.size() > 1) raw_path.back() = goal;

    // Simplify path using line-of-sight
    path = SimplifyPath(raw_path);
    return true;
}

std::vector<Eigen::Vector3d> TrajectoryManager::SimplifyPath(
    const std::vector<Eigen::Vector3d> &input) const {
    if (input.size() <= 2) return input;

    std::vector<Eigen::Vector3d> result;
    result.push_back(input.front());

    size_t i = 0;
    while (i < input.size() - 1) {
        size_t j = i + 1;
        for (size_t k = input.size() - 1; k > i + 1; --k) {
            if (IsSegmentFree(input[i], input[k])) {
                j = k;
                break;
            }
        }
        result.push_back(input[j]);
        i = j;
    }
    return result;
}

// ============================================================
// Build Safe Flight Corridors (AABB per path segment)
// ============================================================
bool TrajectoryManager::BuildCorridors(const std::vector<Eigen::Vector3d> &path,
                                        double margin,
                                        std::vector<Corridor> &corridors) {
    corridors.clear();
    if (path.size() < 2) return false;

    int n_segs = static_cast<int>(path.size()) - 1;
    for (int seg = 0; seg < n_segs; ++seg) {
        const Eigen::Vector3d &ps = path[seg];
        const Eigen::Vector3d &pe = path[seg + 1];

        // Seed AABB: tight box around segment
        double x_min = std::min(ps.x(), pe.x()) - grid_res_ * 0.5;
        double x_max = std::max(ps.x(), pe.x()) + grid_res_ * 0.5;
        double y_min = std::min(ps.y(), pe.y()) - grid_res_ * 0.5;
        double y_max = std::max(ps.y(), pe.y()) + grid_res_ * 0.5;
        double z_min = std::min(ps.z(), pe.z()) - grid_res_ * 0.5;
        double z_max = std::max(ps.z(), pe.z()) + grid_res_ * 0.5;

        // Expand each face until hitting occupied or boundary
        const int MAX_EXPAND = 200;
        // Expand -x
        for (int s = 0; s < MAX_EXPAND; ++s) {
            double cand = x_min - grid_res_;
            bool free = true;
            Eigen::Vector3i lo = WorldToGrid(Eigen::Vector3d(cand, y_min, z_min));
            Eigen::Vector3i hi = WorldToGrid(Eigen::Vector3d(cand, y_max, z_max));
            for (int iy = std::max(lo.y(), 0); iy <= std::min(hi.y(), grid_ny_-1) && free; ++iy)
                for (int iz = std::max(lo.z(), 0); iz <= std::min(hi.z(), grid_nz_-1) && free; ++iz)
                    if (IsOccupied(lo.x(), iy, iz)) free = false;
            if (!free || cand < WS_X_MIN) break;
            x_min = cand;
        }
        // Expand +x
        for (int s = 0; s < MAX_EXPAND; ++s) {
            double cand = x_max + grid_res_;
            bool free = true;
            Eigen::Vector3i lo = WorldToGrid(Eigen::Vector3d(cand, y_min, z_min));
            Eigen::Vector3i hi = WorldToGrid(Eigen::Vector3d(cand, y_max, z_max));
            for (int iy = std::max(lo.y(), 0); iy <= std::min(hi.y(), grid_ny_-1) && free; ++iy)
                for (int iz = std::max(lo.z(), 0); iz <= std::min(hi.z(), grid_nz_-1) && free; ++iz)
                    if (IsOccupied(hi.x(), iy, iz)) free = false;
            if (!free || cand > WS_X_MAX) break;
            x_max = cand;
        }
        // Expand -y
        for (int s = 0; s < MAX_EXPAND; ++s) {
            double cand = y_min - grid_res_;
            bool free = true;
            Eigen::Vector3i lo = WorldToGrid(Eigen::Vector3d(x_min, cand, z_min));
            Eigen::Vector3i hi = WorldToGrid(Eigen::Vector3d(x_max, cand, z_max));
            for (int ix = std::max(lo.x(), 0); ix <= std::min(hi.x(), grid_nx_-1) && free; ++ix)
                for (int iz = std::max(lo.z(), 0); iz <= std::min(hi.z(), grid_nz_-1) && free; ++iz)
                    if (IsOccupied(ix, lo.y(), iz)) free = false;
            if (!free || cand < WS_Y_MIN) break;
            y_min = cand;
        }
        // Expand +y
        for (int s = 0; s < MAX_EXPAND; ++s) {
            double cand = y_max + grid_res_;
            bool free = true;
            Eigen::Vector3i lo = WorldToGrid(Eigen::Vector3d(x_min, cand, z_min));
            Eigen::Vector3i hi = WorldToGrid(Eigen::Vector3d(x_max, cand, z_max));
            for (int ix = std::max(lo.x(), 0); ix <= std::min(hi.x(), grid_nx_-1) && free; ++ix)
                for (int iz = std::max(lo.z(), 0); iz <= std::min(hi.z(), grid_nz_-1) && free; ++iz)
                    if (IsOccupied(ix, hi.y(), iz)) free = false;
            if (!free || cand > WS_Y_MAX) break;
            y_max = cand;
        }
        // Expand -z
        for (int s = 0; s < MAX_EXPAND; ++s) {
            double cand = z_min - grid_res_;
            bool free = true;
            Eigen::Vector3i lo = WorldToGrid(Eigen::Vector3d(x_min, y_min, cand));
            Eigen::Vector3i hi = WorldToGrid(Eigen::Vector3d(x_max, y_max, cand));
            for (int ix = std::max(lo.x(), 0); ix <= std::min(hi.x(), grid_nx_-1) && free; ++ix)
                for (int iy = std::max(lo.y(), 0); iy <= std::min(hi.y(), grid_ny_-1) && free; ++iy)
                    if (IsOccupied(ix, iy, lo.z())) free = false;
            if (!free || cand < WS_Z_MIN) break;
            z_min = cand;
        }
        // Expand +z
        for (int s = 0; s < MAX_EXPAND; ++s) {
            double cand = z_max + grid_res_;
            bool free = true;
            Eigen::Vector3i lo = WorldToGrid(Eigen::Vector3d(x_min, y_min, cand));
            Eigen::Vector3i hi = WorldToGrid(Eigen::Vector3d(x_max, y_max, cand));
            for (int ix = std::max(lo.x(), 0); ix <= std::min(hi.x(), grid_nx_-1) && free; ++ix)
                for (int iy = std::max(lo.y(), 0); iy <= std::min(hi.y(), grid_ny_-1) && free; ++iy)
                    if (IsOccupied(ix, iy, hi.z())) free = false;
            if (!free || cand > WS_Z_MAX) break;
            z_max = cand;
        }

        // Shrink by safety margin
        x_min += margin;
        x_max -= margin;
        y_min += margin;
        y_max -= margin;
        z_min += margin;
        z_max -= margin;

        // Check corridor didn't collapse
        if (x_min >= x_max || y_min >= y_max || z_min >= z_max) {
            Warn("corridor collapsed for segment %d\n", seg);
            return false;
        }

        Corridor c;
        c.lo = Eigen::Vector3d(x_min, y_min, z_min);
        c.hi = Eigen::Vector3d(x_max, y_max, z_max);
        corridors.push_back(c);
    }

    // Ensure consecutive corridors overlap at shared waypoints
    for (int i = 0; i < static_cast<int>(corridors.size()) - 1; ++i) {
        const Eigen::Vector3d &wp = path[i + 1];
        // Expand corridor i to contain wp
        for (int a = 0; a < 3; ++a) {
            if (corridors[i].lo(a) > wp(a)) corridors[i].lo(a) = wp(a) - 1e-6;
            if (corridors[i].hi(a) < wp(a)) corridors[i].hi(a) = wp(a) + 1e-6;
        }
        // Expand corridor i+1 to contain wp
        for (int a = 0; a < 3; ++a) {
            if (corridors[i+1].lo(a) > wp(a)) corridors[i+1].lo(a) = wp(a) - 1e-6;
            if (corridors[i+1].hi(a) < wp(a)) corridors[i+1].hi(a) = wp(a) + 1e-6;
        }
    }

    return true;
}

// ============================================================
// Corridor-constrained min-snap (iterative project-and-insert)
// ============================================================
bool TrajectoryManager::SolveMinSnapConstrained(
    const std::vector<Eigen::Vector3d> &initial_waypoints,
    const std::vector<Corridor> &corridors) {

    // Working copy of waypoints
    std::vector<Eigen::Vector3d> wps = initial_waypoints;

    // Corridor assignment: corridor_map[seg_i] = index into corridors
    std::vector<int> corridor_map;
    {
        int M = static_cast<int>(wps.size()) - 1;
        corridor_map.resize(M);
        for (int i = 0; i < M; ++i) {
            corridor_map[i] = std::min(i, static_cast<int>(corridors.size()) - 1);
        }
    }

    const int MAX_ITER = 3;
    const int K = 10;  // check points per segment

    for (int iter = 0; iter < MAX_ITER; ++iter) {
        int M = static_cast<int>(wps.size()) - 1;
        if (M < 1 || M > MAX_SEGMENTS) return false;

        // Set waypoints for SolveMinSnap
        num_waypoints_ = static_cast<int>(wps.size());
        for (int i = 0; i < num_waypoints_; ++i) {
            waypoints_[i] = wps[i];
        }

        if (!SolveMinSnap()) return false;

        // Check corridor violations at K sample points per segment
        double worst_viol = 0.0;
        int worst_seg = -1;
        double worst_t = 0.0;
        Eigen::Vector3d worst_pos = Eigen::Vector3d::Zero();

        double t_acc = 0.0;
        for (int seg = 0; seg < num_segments_; ++seg) {
            int ci = corridor_map[seg];
            const Corridor &corr = corridors[ci];

            for (int k = 0; k < K; ++k) {
                double t_frac = (K > 1) ? (static_cast<double>(k) / (K - 1)) : 0.5;
                double t_sample = t_acc + t_frac * segments_[seg].duration;
                Eigen::Vector3d p = EvalPos(t_sample);

                for (int a = 0; a < 3; ++a) {
                    double viol = 0.0;
                    if (p(a) < corr.lo(a)) viol = corr.lo(a) - p(a);
                    else if (p(a) > corr.hi(a)) viol = p(a) - corr.hi(a);
                    if (viol > worst_viol) {
                        worst_viol = viol;
                        worst_seg = seg;
                        worst_t = t_sample;
                        worst_pos = p;
                    }
                }
            }
            t_acc += segments_[seg].duration;
        }

        // No violations -> done
        if (worst_viol < 1e-3) return true;

        // Insert new waypoint clamped to corridor
        int ci = corridor_map[worst_seg];
        const Corridor &corr = corridors[ci];
        Eigen::Vector3d new_wp;
        for (int a = 0; a < 3; ++a) {
            new_wp(a) = std::max(corr.lo(a), std::min(worst_pos(a), corr.hi(a)));
        }

        // Insert after segment start
        int insert_pos = worst_seg + 1;
        wps.insert(wps.begin() + insert_pos, new_wp);
        corridor_map.insert(corridor_map.begin() + worst_seg + 1, ci);
    }

    // Final solve with all inserted waypoints
    int M = static_cast<int>(wps.size()) - 1;
    if (M < 1 || M > MAX_SEGMENTS) return false;
    num_waypoints_ = static_cast<int>(wps.size());
    for (int i = 0; i < num_waypoints_; ++i) {
        waypoints_[i] = wps[i];
    }
    return SolveMinSnap();
}

// ============================================================
// Full obstacle avoidance pipeline
// ============================================================
bool TrajectoryManager::PlanWithObstacleAvoidance() {
    // Step 1: Build occupancy grid
    BuildOccupancyGrid();

    double sm = safety_margin_->Value();

    // Step 2: Find collision-free path through consecutive waypoints
    std::vector<Eigen::Vector3d> full_path;
    full_path.push_back(waypoints_[0]);

    for (int i = 0; i < num_waypoints_ - 1; ++i) {
        std::vector<Eigen::Vector3d> seg_path;
        if (!FindPath(waypoints_[i], waypoints_[i + 1], seg_path)) {
            Warn("A* failed between WP%d and WP%d\n", i, i + 1);
            // Fallback: try direct connection
            seg_path.clear();
            seg_path.push_back(waypoints_[i]);
            seg_path.push_back(waypoints_[i + 1]);
        }
        // Append (skip first to avoid duplicates)
        for (size_t j = 1; j < seg_path.size(); ++j) {
            full_path.push_back(seg_path[j]);
        }
    }

    // Limit path waypoints to avoid exceeding MAX_WAYPOINTS
    if (static_cast<int>(full_path.size()) > MAX_WAYPOINTS) {
        // Subsample: keep first, last, and evenly spaced points
        std::vector<Eigen::Vector3d> sampled;
        sampled.push_back(full_path.front());
        int n = static_cast<int>(full_path.size());
        int keep = MAX_WAYPOINTS - 2;
        for (int i = 1; i <= keep; ++i) {
            int idx = static_cast<int>(static_cast<double>(i) / (keep + 1) * (n - 1));
            if (idx > 0 && idx < n - 1) sampled.push_back(full_path[idx]);
        }
        sampled.push_back(full_path.back());
        full_path = sampled;
    }

    Info("obstacle avoidance: path has %d waypoints\n", static_cast<int>(full_path.size()));

    // Step 3: Build SFC corridors around path
    std::vector<Corridor> corridors;
    if (!BuildCorridors(full_path, sm, corridors)) {
        Warn("corridor building failed, falling back to unconstrained solve\n");
        // Fallback: use the path waypoints with unconstrained solver
        num_waypoints_ = static_cast<int>(full_path.size());
        for (int i = 0; i < num_waypoints_; ++i) {
            waypoints_[i] = full_path[i];
        }
        return SolveMinSnap();
    }

    // Step 4: Corridor-constrained min-snap
    return SolveMinSnapConstrained(full_path, corridors);
}
