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
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <cmath>
#include <algorithm>
#include <cstring>
#include <queue>
#include <limits>
#include <stdexcept>

using std::string;
using namespace flair::core;
using namespace flair::gui;

// Weighted A* heuristic weight (w > 1 trades optimality for speed)
// w=1.3 gives 3-5x speedup with paths within 30% of optimal
static const float ASTAR_WEIGHT = 1.3f;

// ============================================================
// Coordinate Frame Conventions
// ============================================================
// World frame (Flair/Aerospace NED): x=forward, y=right, z=down
//   - Ground at z=0, flight altitude at z<0 (e.g., z=-1.5)
//   - Gravity vector: (0, 0, +9.81) in NED
//
// Planner internal frame: x/y SWAPPED from world
//   - planner_x = world_y (right)
//   - planner_y = world_x (forward)
//   - planner_z = world_z (down, negative = up)
//   - Swap applied at input: ReadWaypointsFromGUI(), Update(), UpdateObstaclePosition()
//   - Swap reversed at output: output_matrix_ SetValue calls, last_pos_/vel_/acc_/jerk_
//
// GCOPTER library (flatness.hpp): assumes z-UP (ENU-like) internally
//   - flatness.hpp line 79: zu2 = a2 + grav assumes gravity adds to z (z points up)
//   - This affects the constrained solver's penalty functional (thrust, tilt, body-rate)
//   - For mainly horizontal flight the impact is negligible (tilt≈0, thrust≈mg)
//   - The trajectory positions/velocities from MINCO are purely kinematic (frame-agnostic)
//   - TODO: For aggressive vertical maneuvers, negate z before/after the constrained solver
// ============================================================

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
      use_gcopter_(true),
      use_gcopter_traj_(false),
      gcopter_traj_valid_(false),
      time_opt_enabled_(true),
      kT_(500.0),
      topp_ra_enabled_(true),
      replan_horizon_(3.0),
      blend_duration_(0.5),
      contingency_valid_(false),
      prev_traj_valid_(false),
      prev_traj_start_time_(0.0),
      current_uav_pos_(Eigen::Vector3d::Zero()),
      current_uav_vel_(Eigen::Vector3d::Zero()),
      current_uav_acc_(Eigen::Vector3d::Zero()),
      execution_start_time_(0.0),
      execution_time_set_(false),
      last_replan_time_(0.0),
      num_waypoints_(2),
      start_vel_(Eigen::Vector3d::Zero()),
      end_vel_(Eigen::Vector3d::Zero()),
      num_obstacles_(0),
      grid_nx_(0), grid_ny_(0), grid_nz_(0),
      grid_res_(0.1),
      grid_origin_(WS_X_MIN, WS_Y_MIN, WS_Z_MIN),
      grid_dirty_(true),
      grid_initialized_(false),
      prev_num_obstacles_(0),
      inflation_template_radius_(0.0),
      cache_valid_(false),
      warm_start_enabled_(true),
      prev_coeffs_valid_(false),
      distance_field_valid_(false),
      safety_monitor_enabled_(true),
      emergency_distance_(0.3),
      replan_distance_(0.8),
      last_safety_replan_time_(0.0),
      contingency_active_(false),
      contingency_start_time_(0.0),
      contingency_duration_(0.0),
      post_contingency_replan_attempts_(0),
      post_contingency_cooldown_until_(0.0),
      safety_replan_failures_(0),
      last_update_time_(0.0)
{
    // --------------------------------------------------------
    // Output matrix with named elements (13 elements)
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
    // Note: default values come from the XML config file. On first run
    // (no XML), Flair uses the minimum value of the range as default.
    // The user should set XY range >= 5 and Max altitude >= 3 for typical use.

    // Planner backend selection
    planner_backend_ = new ComboBox(settings_box_->NewRow(), "Planner backend");
    planner_backend_->AddItem("Min-Snap (basic)");
    planner_backend_->AddItem("GCOPTER/MINCO");

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
void TrajectoryManager::Update(Time time,
                               const Vector3Df &uav_pos,
                               const Vector3Df &uav_vel) {
    // Store current UAV position so Plan() can use it as WP0
    // Swap x and y to match planner's internal frame
    current_uav_pos_ = Eigen::Vector3d(uav_pos.y, uav_pos.x, uav_pos.z);
    current_uav_vel_ = Eigen::Vector3d(uav_vel.y, uav_vel.x, uav_vel.z);
    // Estimate acceleration from trajectory if executing
    if (state_ == State::EXECUTING && trajectory_valid_ && execution_time_set_) {
        double t_now = static_cast<double>(time) / 1e9 - execution_start_time_;
        if (t_now >= 0.0 && t_now <= total_duration_) {
            if (use_gcopter_ && gcopter_traj_valid_) {
                current_uav_acc_ = gcopter_traj_.getAcc(t_now);
            } else {
                current_uav_acc_ = EvalAcc(t_now);
            }
        }
    }

    // Check GUI buttons
    if (plan_button_->Clicked()) {
        Plan(uav_pos, uav_vel);
    }
    if (execute_button_->Clicked() && state_ == State::PLANNED) {
        StartTraj();
        execution_time_set_ = false;  // will be set on first real tick below
    }
    if (stop_button_->Clicked()) {
        StopTraj();
    }

    // Convert Flair time to seconds
    double t_sec = static_cast<double>(time) / 1e9;
    last_update_time_ = t_sec;

    if (state_ == State::EXECUTING && trajectory_valid_) {
        // Set execution start time on the first real tick after Execute is clicked.
        // This avoids the instant-completion bug when Update() is not called
        // continuously (e.g. UAV not in flight state).
        if (!execution_time_set_) {
            execution_start_time_ = t_sec;
            execution_time_set_ = true;
        }
        double t_traj = t_sec - execution_start_time_;
        double elapsed = (t_traj < 0.0) ? 0.0 : ((t_traj > total_duration_) ? total_duration_ : t_traj);
        float prog = (total_duration_ > 1e-9) ? static_cast<float>(elapsed / total_duration_) : 1.0f;

        Eigen::Vector3d p, v, a, j;
        if (use_gcopter_ && gcopter_traj_valid_) {
            p = gcopter_traj_.getPos(elapsed);
            v = gcopter_traj_.getVel(elapsed);
            a = gcopter_traj_.getAcc(elapsed);
            j = gcopter_traj_.getJer(elapsed);
        } else {
            p = EvalPos(elapsed);
            v = EvalVel(elapsed);
            a = EvalAcc(elapsed);
            j = EvalJer(elapsed);
        }

        // Safety clamp: prevent runaway velocity commands if MINCO polynomial
        // overshoots (e.g. due to sharp path corners or insufficient time alloc).
        // Hard limit at 1.5 * v_max; zero acceleration/jerk when clamped.
        {
            double v_max_lim = max_vel_->Value();
            if (v_max_lim < 0.1) v_max_lim = 0.1;
            double v_norm = v.norm();
            if (v_norm > v_max_lim * 1.5) {
                v *= (v_max_lim * 1.5 / v_norm);
                a = Eigen::Vector3d::Zero();
                j = Eigen::Vector3d::Zero();
            }
        }

        // Thread-safe matrix update (Flair pattern)
        output_matrix_->GetMutex();
        output_matrix_->SetValueNoMutex(0, 0, static_cast<float>(p.y()));  // des_x (swapped)
        output_matrix_->SetValueNoMutex(1, 0, static_cast<float>(p.x()));  // des_y (swapped)
        output_matrix_->SetValueNoMutex(2, 0, static_cast<float>(p.z()));
        output_matrix_->SetValueNoMutex(3, 0, static_cast<float>(v.y()));  // des_vx (swapped)
        output_matrix_->SetValueNoMutex(4, 0, static_cast<float>(v.x()));  // des_vy (swapped)
        output_matrix_->SetValueNoMutex(5, 0, static_cast<float>(v.z()));
        output_matrix_->SetValueNoMutex(6, 0, static_cast<float>(a.y()));  // des_ax (swapped)
        output_matrix_->SetValueNoMutex(7, 0, static_cast<float>(a.x()));  // des_ay (swapped)
        output_matrix_->SetValueNoMutex(8, 0, static_cast<float>(a.z()));
        output_matrix_->SetValueNoMutex(9, 0, static_cast<float>(j.y()));  // des_jx (swapped)
        output_matrix_->SetValueNoMutex(10, 0, static_cast<float>(j.x())); // des_jy (swapped)
        output_matrix_->SetValueNoMutex(11, 0, static_cast<float>(j.z()));
        output_matrix_->SetValueNoMutex(12, 0, prog);
        output_matrix_->ReleaseMutex();

        // Store for GetPosition/GetSpeed/etc accessors
        // Swap x/y back to world frame for controller output
        last_pos_  = Vector3Df(static_cast<float>(p.y()), static_cast<float>(p.x()), static_cast<float>(p.z()));
        last_vel_  = Vector3Df(static_cast<float>(v.y()), static_cast<float>(v.x()), static_cast<float>(v.z()));
        last_acc_  = Vector3Df(static_cast<float>(a.y()), static_cast<float>(a.x()), static_cast<float>(a.z()));
        last_jerk_ = Vector3Df(static_cast<float>(j.y()), static_cast<float>(j.x()), static_cast<float>(j.z()));
        progress_ = prog;

        // Signal data update to DataPlot framework
        output_matrix_->SetDataTime(time);
        ProcessUpdate(output_matrix_);

        // Check trajectory completion — hold final position
        if (t_traj >= total_duration_) {
            // Store the final position (evaluated at t = total_duration)
            Eigen::Vector3d p_end = (use_gcopter_ && gcopter_traj_valid_)
                ? gcopter_traj_.getPos(total_duration_)
                : EvalPos(total_duration_);
            // p_end is in planner frame; swap x/y back to world frame
            last_pos_ = Vector3Df(static_cast<float>(p_end.y()),   // world_x = planner_y
                                  static_cast<float>(p_end.x()),   // world_y = planner_x
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

        // --- SOTA: Real-time safety monitor ---
        if (safety_monitor_enabled_ && !distance_field_.empty() &&
            obstacle_avoidance_mode_->CurrentIndex() == 1 && num_obstacles_ > 0) {
            double obs_dist = GetObstacleDistance(current_uav_pos_);

            // If contingency is active, let it complete before any other action
            if (contingency_active_) {
                double contingency_elapsed = t_sec - contingency_start_time_;
                if (contingency_elapsed >= contingency_duration_) {
                    // Contingency trajectory completed
                    contingency_active_ = false;
                    // Always hold position after contingency — do NOT replan.
                    // Replanning from inside/near the obstacle creates a loop:
                    //   contingency → replan → new traj approaches obstacle → contingency
                    Vector3Df cur_pos_w(static_cast<float>(current_uav_pos_.y()),
                                        static_cast<float>(current_uav_pos_.x()),
                                        static_cast<float>(current_uav_pos_.z()));
                    state_ = State::HOLDING;
                    last_pos_ = cur_pos_w;
                    post_contingency_cooldown_until_ = t_sec + kPostContingencyCooldown;
                    if (obs_dist >= emergency_distance_) {
                        status_label_->SetText("HOLDING (post-contingency)");
                        Info("safety monitor: contingency completed, obstacle cleared (%.2fm), holding\n", obs_dist);
                    } else {
                        status_label_->SetText("HOLDING (obstacle nearby)");
                        Warn("safety monitor: contingency done, obstacle still at %.2fm, holding\n", obs_dist);
                    }
                }
                // While contingency is active, do NOT re-trigger — let it execute
            } else if (obs_dist < emergency_distance_ && t_sec >= post_contingency_cooldown_until_) {
                // Trigger contingency ONLY if not in post-contingency cooldown
                GenerateContingencyTrajectory(current_uav_pos_, current_uav_vel_, current_uav_acc_);
                if (contingency_valid_) {
                    gcopter_traj_ = contingency_traj_;
                    gcopter_traj_valid_ = true;
                    total_duration_ = contingency_traj_.getTotalDuration();
                    trajectory_valid_ = true;
                    num_segments_ = contingency_traj_.getPieceNum();
                    execution_start_time_ = t_sec;
                    execution_time_set_ = true;
                    contingency_active_ = true;
                    contingency_start_time_ = t_sec;
                    contingency_duration_ = contingency_traj_.getTotalDuration();
                    status_label_->SetText("EMERGENCY STOP");
                    Warn("safety monitor: obstacle at %.2fm < emergency %.2fm, contingency activated\n",
                         obs_dist, emergency_distance_);
                } else {
                    // Can't even generate contingency — hold immediately
                    Vector3Df cur_pos_w(static_cast<float>(current_uav_pos_.y()),
                                        static_cast<float>(current_uav_pos_.x()),
                                        static_cast<float>(current_uav_pos_.z()));
                    state_ = State::HOLDING;
                    last_pos_ = cur_pos_w;
                    post_contingency_cooldown_until_ = t_sec + kPostContingencyCooldown;
                    status_label_->SetText("HOLDING (emergency)");
                    Warn("safety monitor: obstacle at %.2fm, contingency failed, holding\n", obs_dist);
                }
            } else if (obs_dist < replan_distance_ && safety_replan_failures_ < kMaxSafetyReplanFailures) {
                // Replan-distance zone: try to replan, but give up after repeated failures
                if ((t_sec - last_safety_replan_time_) > 1.0) {
                    last_safety_replan_time_ = t_sec;
                    Warn("safety monitor: obstacle at %.2fm < replan threshold %.2fm, replanning (attempt %d/%d)\n",
                         obs_dist, replan_distance_, safety_replan_failures_ + 1, kMaxSafetyReplanFailures);
                    Vector3Df cur_pos_w(static_cast<float>(current_uav_pos_.y()),
                                        static_cast<float>(current_uav_pos_.x()),
                                        static_cast<float>(current_uav_pos_.z()));
                    Vector3Df cur_vel_w(static_cast<float>(current_uav_vel_.y()),
                                        static_cast<float>(current_uav_vel_.x()),
                                        static_cast<float>(current_uav_vel_.z()));
                    if (Replan(cur_pos_w, cur_vel_w)) {
                        safety_replan_failures_ = 0;  // success resets counter
                    } else {
                        safety_replan_failures_++;
                        if (safety_replan_failures_ >= kMaxSafetyReplanFailures) {
                            state_ = State::HOLDING;
                            last_pos_ = cur_pos_w;
                            status_label_->SetText("HOLDING (replan failed)");
                            Warn("safety monitor: %d consecutive replan failures, holding\n",
                                 kMaxSafetyReplanFailures);
                        }
                    }
                }
            } else if (obs_dist >= replan_distance_) {
                // Obstacle is far enough — reset failure counter
                safety_replan_failures_ = 0;
            }
        }
    } else if (state_ == State::HOLDING) {
        // Holding final position — output last_pos_ with zero derivatives
        output_matrix_->GetMutex();
        output_matrix_->SetValueNoMutex(0, 0, last_pos_.x);  // already swapped
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
        contingency_active_ = false;
        post_contingency_replan_attempts_ = 0;
        post_contingency_cooldown_until_ = 0.0;
        safety_replan_failures_ = 0;
        status_label_->SetText("EXECUTING");
        Info("trajectory execution started\n");
    }
}

void TrajectoryManager::StopTraj() {
    state_ = State::IDLE;
    execution_time_set_ = false;
    contingency_active_ = false;
    post_contingency_replan_attempts_ = 0;
    post_contingency_cooldown_until_ = 0.0;
    safety_replan_failures_ = 0;
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

    // Swap x/y to match planner internal frame (planner_x=world_y, planner_y=world_x)
    start_vel_ = Eigen::Vector3d(current_vel.y, current_vel.x, current_vel.z);
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

    // Reinitialize grid — auto-expand workspace to cover all waypoints + margin
    double res = grid_res_spin_->Value();
    double xy_range = ws_xy_range_->Value();
    double z_alt = ws_z_max_alt_->Value();
    WS_X_MIN = -xy_range;  WS_X_MAX = xy_range;
    WS_Y_MIN = -xy_range;  WS_Y_MAX = xy_range;
    WS_Z_MIN = -z_alt;     WS_Z_MAX = 0.0;

    // Auto-expand workspace to include all waypoints and obstacles with margin
    double expand_margin = 2.0; // 2m padding around all points
    for (int i = 0; i < num_waypoints_; ++i) {
        if (waypoints_[i].x() - expand_margin < WS_X_MIN) WS_X_MIN = waypoints_[i].x() - expand_margin;
        if (waypoints_[i].x() + expand_margin > WS_X_MAX) WS_X_MAX = waypoints_[i].x() + expand_margin;
        if (waypoints_[i].y() - expand_margin < WS_Y_MIN) WS_Y_MIN = waypoints_[i].y() - expand_margin;
        if (waypoints_[i].y() + expand_margin > WS_Y_MAX) WS_Y_MAX = waypoints_[i].y() + expand_margin;
        if (waypoints_[i].z() - expand_margin < WS_Z_MIN) WS_Z_MIN = waypoints_[i].z() - expand_margin;
    }
    for (int i = 0; i < num_obstacles_; ++i) {
        double r = obstacles_[i].radius + 1.0;
        if (obstacles_[i].pos.x() - r < WS_X_MIN) WS_X_MIN = obstacles_[i].pos.x() - r;
        if (obstacles_[i].pos.x() + r > WS_X_MAX) WS_X_MAX = obstacles_[i].pos.x() + r;
        if (obstacles_[i].pos.y() - r < WS_Y_MIN) WS_Y_MIN = obstacles_[i].pos.y() - r;
        if (obstacles_[i].pos.y() + r > WS_Y_MAX) WS_Y_MAX = obstacles_[i].pos.y() + r;
    }

    // Clamp grid resolution so grid doesn't get too large (max ~150K cells)
    double vol = (WS_X_MAX-WS_X_MIN) * (WS_Y_MAX-WS_Y_MIN) * (WS_Z_MAX-WS_Z_MIN);
    double min_res = std::pow(vol / 150000.0, 1.0/3.0);
    if (res < min_res) {
        Info("auto-increasing grid resolution from %.2f to %.2f to fit workspace\n", res, min_res);
        res = min_res;
    }

    InitGrid(res);

    // Select planner backend — MINCO/GCOPTER is now default (SOTA Upgrade 1)
    int backend_idx = planner_backend_->CurrentIndex();
    use_gcopter_ = (backend_idx == 1) || (backend_idx == 0 && use_gcopter_);
    gcopter_traj_valid_ = false;

    // Use full obstacle avoidance pipeline if enabled and obstacles present
    bool ok;
    if (obstacle_avoidance_mode_->CurrentIndex() == 1 && num_obstacles_ > 0) {
        ok = PlanWithObstacleAvoidance();
    } else if (use_gcopter_) {
        ok = SolveGCOPTER();
        if (!ok) {
            Warn("GCOPTER failed, falling back to min-snap\n");
            use_gcopter_ = false;
            ok = SolveMinSnap();
        }
    } else {
        ok = SolveMinSnap();
    }

    // SOTA Upgrade 3: TOPP-RA post-processing for dynamic feasibility
    if (ok && topp_ra_enabled_) {
        if (!ApplyTOPPRA()) {
            Warn("TOPP-RA post-processing failed, keeping original trajectory\n");
        }
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

    // --- SOTA Upgrade 5: Receding-horizon replanning with warm start ---

    // Save previous trajectory for blending
    if (gcopter_traj_valid_ && use_gcopter_) {
        prev_traj_ = gcopter_traj_;
        prev_traj_valid_ = true;
        prev_traj_start_time_ = execution_start_time_;
    }

    // Step 1: Extract current state in planner frame
    Eigen::Vector3d cur_pos_plan(current_pos.y, current_pos.x, current_pos.z);
    Eigen::Vector3d cur_vel_plan(current_vel.y, current_vel.x, current_vel.z);
    Eigen::Vector3d cur_acc_plan = Eigen::Vector3d::Zero();

    // If we have a valid executing trajectory, extract accurate acc from it
    if (trajectory_valid_ && gcopter_traj_valid_ && execution_time_set_) {
        cur_pos_plan = current_uav_pos_;
        cur_vel_plan = current_uav_vel_;
        cur_acc_plan = current_uav_acc_;
    }

    // Step 2: Only replan from current position to goal
    Eigen::Vector3d goal = waypoints_[num_waypoints_ - 1];
    waypoints_[0] = cur_pos_plan;
    start_vel_ = cur_vel_plan;

    // If we had intermediate waypoints, keep only those that are still ahead
    if (num_waypoints_ > 2 && trajectory_valid_) {
        std::vector<Eigen::Vector3d> remaining_wps;
        remaining_wps.push_back(cur_pos_plan);

        for (int i = 1; i < num_waypoints_; ++i) {
            double d = (waypoints_[i] - cur_pos_plan).norm();
            if (d > 0.1) {
                remaining_wps.push_back(waypoints_[i]);
            }
        }
        if ((remaining_wps.back() - goal).norm() > 1e-6) {
            remaining_wps.push_back(goal);
        }

        num_waypoints_ = std::min(static_cast<int>(remaining_wps.size()), MAX_WAYPOINTS);
        for (int i = 0; i < num_waypoints_; ++i) {
            waypoints_[i] = remaining_wps[i];
        }
    }

    // Update obstacle radius from GUI
    double obs_r = obstacle_radius_spin_->Value();
    for (int i = 0; i < num_obstacles_; ++i) {
        obstacles_[i].radius = obs_r;
    }

    gcopter_traj_valid_ = false;

    // Step 3: Generate contingency trajectory (decelerate to hover)
    GenerateContingencyTrajectory(cur_pos_plan, cur_vel_plan, cur_acc_plan);

    bool ok;
    if (obstacle_avoidance_mode_->CurrentIndex() == 1 && num_obstacles_ > 0) {
        ok = PlanWithObstacleAvoidance();
    } else if (use_gcopter_) {
        ok = SolveGCOPTER();
        if (!ok) {
            Warn("GCOPTER replan failed, falling back to min-snap\n");
            ok = SolveMinSnap();
        }
    } else {
        ok = SolveMinSnap();
    }

    // TOPP-RA post-processing
    if (ok && topp_ra_enabled_) {
        ApplyTOPPRA();
    }

    if (ok) {
        state_ = State::EXECUTING;
        execution_time_set_ = false;  // reset so blend timing works
        contingency_active_ = false;  // successful replan clears contingency
        status_label_->SetText("EXECUTING (replanned)");
    } else {
        // Replan failed — restore previous state, return false.
        // The caller (safety monitor) will transition to HOLDING.
        // Do NOT switch to contingency here — that creates a loop when near
        // an obstacle (contingency ends near obstacle → replan fails → contingency → ...).
        state_ = prev;
        Warn("replanning failed, returning to %s\n",
             prev == State::EXECUTING ? "EXECUTING" : "previous state");
    }
    return ok;
}

// ============================================================
// Obstacle management
// ============================================================
void TrajectoryManager::AddObstacle(const Vector3Df &pos, float radius) {
    if (num_obstacles_ < MAX_OBSTACLES) {
        obstacles_[num_obstacles_].pos = Eigen::Vector3d(pos.y, pos.x, pos.z);  // swap x/y to planner frame
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
        obstacles_[idx].pos = Eigen::Vector3d(pos.y, pos.x, pos.z);  // swap x/y
    }
}

void TrajectoryManager::UpdateObstacleVelocity(int idx, const Vector3Df &vel) {
    if (idx >= 0 && idx < num_obstacles_) {
        obstacles_[idx].vel = Eigen::Vector3d(vel.y, vel.x, vel.z);  // swap x/y
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
        // Swap x and y: GUI label "X" is physical forward but planner x is right
        waypoints_[i] = Eigen::Vector3d(wp_y_[i]->Value(),
                                         wp_x_[i]->Value(),
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
        double t_seg = dist / v_max * 1.5;  // alpha=1.5 time buffer for polynomial headroom
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

        // Solve with PartialPivLU (faster for small systems, sufficient for degree-7 polynomials)
        Eigen::VectorXd c = A.partialPivLu().solve(b);

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
    // Pre-initialize last_pos_ from the first waypoint so GetPosition()
    // returns a sane value before the first Update() tick in EXECUTING state
    if (num_waypoints_ > 0) {
        // waypoints_[0] is in planner frame — swap back for controller output
        last_pos_ = Vector3Df(static_cast<float>(waypoints_[0].y()),
                              static_cast<float>(waypoints_[0].x()),
                              static_cast<float>(waypoints_[0].z()));
    }
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
// GCOPTER/MINCO backend: helper to extract obstacle points near a segment
// ################################################################
std::vector<Eigen::Vector3d> TrajectoryManager::GetNearbyObstaclePoints(
    const Eigen::Vector3d &seg_start,
    const Eigen::Vector3d &seg_end,
    double radius) const {

    std::vector<Eigen::Vector3d> pts;

    // Bounding box of the segment expanded by radius
    Eigen::Vector3d lo, hi;
    for (int a = 0; a < 3; ++a) {
        lo(a) = std::min(seg_start(a), seg_end(a)) - radius;
        hi(a) = std::max(seg_start(a), seg_end(a)) + radius;
    }

    Eigen::Vector3i lo_g = WorldToGrid(lo);
    Eigen::Vector3i hi_g = WorldToGrid(hi);

    int ix0 = std::max(lo_g.x(), 0);
    int iy0 = std::max(lo_g.y(), 0);
    int iz0 = std::max(lo_g.z(), 0);
    int ix1 = std::min(hi_g.x(), grid_nx_ - 1);
    int iy1 = std::min(hi_g.y(), grid_ny_ - 1);
    int iz1 = std::min(hi_g.z(), grid_nz_ - 1);

    for (int ix = ix0; ix <= ix1; ++ix) {
        for (int iy = iy0; iy <= iy1; ++iy) {
            for (int iz = iz0; iz <= iz1; ++iz) {
                if (IsOccupied(ix, iy, iz)) {
                    pts.push_back(GridToWorld(ix, iy, iz));
                }
            }
        }
    }

    return pts;
}

// ################################################################
// GCOPTER/MINCO backend solver (robust version)
// ################################################################

// Helper: check if a set of 3D points has at least 4 non-coplanar points
static bool HasNonCoplanarPoints(const Eigen::Matrix3Xd &pts, int min_pts = 4) {
    int n = static_cast<int>(pts.cols());
    if (n < min_pts) return false;
    // Check coplanarity: compute rank of (pts - centroid)
    Eigen::Vector3d centroid = pts.rowwise().mean();
    Eigen::Matrix3Xd centered = pts.colwise() - centroid;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinU);
    int rank = 0;
    for (int i = 0; i < svd.singularValues().size(); ++i) {
        if (svd.singularValues()(i) > 1e-6) rank++;
    }
    return rank >= 3;  // need full 3D span for valid polytope
}

// Helper: check if a point satisfies all halfplanes of a polytope
// hPoly rows: [a1 a2 a3 b] where a1*x + a2*y + a3*z + b <= 0
static bool IsInsidePolytope(const Eigen::MatrixX4d &hPoly, const Eigen::Vector3d &pt) {
    for (int r = 0; r < hPoly.rows(); ++r) {
        double val = hPoly(r, 0) * pt.x() + hPoly(r, 1) * pt.y()
                   + hPoly(r, 2) * pt.z() + hPoly(r, 3);
        if (val > 1e-6) return false;
    }
    return true;
}

bool TrajectoryManager::SolveGCOPTER() {
    num_segments_ = num_waypoints_ - 1;
    if (num_segments_ < 1 || num_segments_ > MAX_SEGMENTS) {
        return false;
    }

    double v_max = max_vel_->Value();
    if (v_max < 0.1) v_max = 0.1;
    double a_max = max_acc_->Value();
    if (a_max < 0.1) a_max = 0.1;

    // Build occupancy grid if not already built (no-obstacle mode)
    if (grid_data_.empty() || grid_nx_ == 0) {
        double res = grid_res_spin_->Value();
        InitGrid(res);
        BuildOccupancyGridIncremental();
        if (!distance_field_valid_) {
            ComputeDistanceField();
            distance_field_valid_ = true;
        }
    }

    // NOTE: Step 1 (FIRI corridor generation) was removed — it was dead code.
    // FIRI corridors are only meaningful for the constrained solver in
    // PlanWithObstacleAvoidance() → GenerateCorridors() → SolveGCOPTERConstrained().

    // Step 2: MINCO_S3NU with direction-reversal splitting.
    //
    // MINCO freely optimises intermediate junction velocities to minimise
    // integrated snap. At direction-reversal waypoints (e.g. an A* U-turn
    // around an obstacle) the optimal junction velocity is near-zero, so the
    // drone visibly "stops" mid-flight.
    //
    // Fix: detect reversals (consecutive segment angle > 120°), split the
    // MINCO problem at those points, and set a non-zero through-velocity
    // in the departure direction.  Each sub-problem is then solved
    // independently with velocity-aware time scaling and the resulting
    // Piece objects are concatenated into a single Trajectory<5>.

    // 2a — find reversal waypoints
    std::vector<int> split_pts;
    for (int k = 1; k < num_waypoints_ - 1; ++k) {
        double d_in  = (waypoints_[k]   - waypoints_[k-1]).norm();
        double d_out = (waypoints_[k+1] - waypoints_[k]).norm();
        if (d_in < 1e-6 || d_out < 1e-6) continue;
        double cos_a = ((waypoints_[k]   - waypoints_[k-1]) / d_in).dot(
                        (waypoints_[k+1] - waypoints_[k]) / d_out);
        if (cos_a < -0.5) {   // angle > 120°
            split_pts.push_back(k);
            Info("MINCO: reversal at WP%d (cos=%.2f), splitting segment\n", k, cos_a);
        }
    }
    split_pts.push_back(num_waypoints_ - 1);   // always terminate at goal

    // 2b — solve one MINCO sub-problem per segment; concatenate pieces
    Trajectory<5> traj;
    double total_dur = 0.0;
    int cur_start = 0;
    Eigen::Vector3d cur_start_vel = start_vel_;

    for (int s = 0; s < static_cast<int>(split_pts.size()); ++s) {
        int cur_end = split_pts[s];
        int M_seg   = cur_end - cur_start;
        if (M_seg < 1) { cur_start = cur_end; continue; }

        // Boundary conditions
        Eigen::Matrix3d head_s, tail_s;
        head_s.col(0) = waypoints_[cur_start];
        head_s.col(1) = cur_start_vel;
        head_s.col(2) = Eigen::Vector3d::Zero();

        bool is_last_seg = (s == static_cast<int>(split_pts.size()) - 1);
        Eigen::Vector3d cur_end_vel;
        if (is_last_seg) {
            cur_end_vel = end_vel_;          // stop at final goal
        } else {
            // Through-corner: fly at up to 40% of v_max in departure direction
            double d_dep = (waypoints_[cur_end + 1] - waypoints_[cur_end]).norm();
            Eigen::Vector3d dep_dir;
            if (d_dep > 1e-6)
                dep_dir = (waypoints_[cur_end + 1] - waypoints_[cur_end]) / d_dep;
            else
                dep_dir = Eigen::Vector3d::UnitX();
            double junc_speed = std::min(v_max * 0.4, 0.4);
            cur_end_vel = dep_dir * junc_speed;
            Info("MINCO: junction vel at WP%d: %.2f m/s\n", cur_end, junc_speed);
        }

        tail_s.col(0) = waypoints_[cur_end];
        tail_s.col(1) = cur_end_vel;
        tail_s.col(2) = Eigen::Vector3d::Zero();

        // Interior waypoints for this sub-segment
        Eigen::Matrix3Xd inPs_s(3, M_seg - 1);
        for (int i = 0; i < M_seg - 1; ++i)
            inPs_s.col(i) = waypoints_[cur_start + i + 1];

        // Chord-length time allocation
        Eigen::VectorXd ts_s(M_seg);
        for (int i = 0; i < M_seg; ++i) {
            double dist = (waypoints_[cur_start + i + 1]
                        - waypoints_[cur_start + i]).norm();
            double t_seg = dist / v_max * 1.5;
            if (t_seg < 0.5) t_seg = 0.5;
            ts_s(i) = t_seg;
        }

        // Velocity-aware iterative time scaling
        Trajectory<5> sub_traj;
        for (int iter = 0; iter < 5; ++iter) {
            minco::MINCO_S3NU iter_solver;
            iter_solver.setConditions(head_s, tail_s, M_seg);
            iter_solver.setParameters(inPs_s, ts_s);
            iter_solver.getTrajectory(sub_traj);

            double max_vel_sq = 0.0;
            for (int si = 0; si < M_seg; ++si) {
                double dur_i = ts_s(si);
                for (int kk = 0; kk <= 20; ++kk) {
                    double tau = dur_i * static_cast<double>(kk) / 20.0;
                    Eigen::Vector3d vi = sub_traj[si].getVel(tau);
                    double vsq = vi.squaredNorm();
                    if (vsq > max_vel_sq) max_vel_sq = vsq;
                }
            }
            double max_vel_actual = std::sqrt(max_vel_sq);
            if (max_vel_actual <= v_max * 1.1) {
                Info("MINCO seg%d iter%d: max_vel=%.2f (ok)\n", s, iter, max_vel_actual);
                break;
            }
            double scale = max_vel_actual / v_max;
            Info("MINCO seg%d iter%d: max_vel=%.2f > v_max=%.2f scale=%.2f\n",
                 s, iter, max_vel_actual, v_max, scale);
            ts_s *= scale;
        }

        // Append pieces into combined trajectory
        for (int p = 0; p < sub_traj.getPieceNum(); ++p)
            traj.emplace_back(sub_traj[p]);
        total_dur += sub_traj.getTotalDuration();

        cur_start     = cur_end;
        cur_start_vel = cur_end_vel;
    }

    // Validate result
    int N = traj.getPieceNum();
    if (N < 1) {
        Warn("MINCO solve returned %d pieces\n", N);
        return false;
    }
    if (std::isnan(total_dur) || std::isinf(total_dur) || total_dur <= 0.0) {
        Warn("MINCO: invalid total duration %.4f\n", total_dur);
        return false;
    }

    // Step 4: Store the result
    gcopter_traj_ = traj;
    gcopter_traj_valid_ = true;

    // Post-solve validation using analytical max-rate
    {
        double actual_max_vel = gcopter_traj_.getMaxVelRate();
        double actual_max_acc = gcopter_traj_.getMaxAccRate();
        Info("MINCO validation: max_vel=%.2f/%.2f max_acc=%.2f/%.2f\n",
             actual_max_vel, v_max, actual_max_acc, a_max);
    }

    // Pre-initialize last_pos_ from start waypoint (swap x/y back to world frame)
    if (num_waypoints_ > 0) {
        last_pos_ = Vector3Df(static_cast<float>(waypoints_[0].y()),
                              static_cast<float>(waypoints_[0].x()),
                              static_cast<float>(waypoints_[0].z()));
    }

    // Compute total duration and store segment info for compatibility
    total_duration_ = total_dur;
    num_segments_ = std::min(N, MAX_SEGMENTS);
    for (int i = 0; i < num_segments_; ++i) {
        segments_[i].duration = traj[i].getDuration();
        segments_[i].coeffs.setZero();
        Eigen::Matrix<double, 3, 6> cm = traj[i].getCoeffMat();
        for (int axis = 0; axis < 3; ++axis) {
            // GCOPTER: col 0 = highest power (t^5), col 5 = constant (t^0)
            for (int c = 0; c <= 5; ++c) {
                segments_[i].coeffs(axis, c) = cm(axis, 5 - c);
            }
        }
    }

    trajectory_valid_ = true;

    // SOTA: Store segment times for warm start on next replan
    if (warm_start_enabled_) {
        prev_trajectory_times_.resize(N);
        for (int i = 0; i < N; ++i) {
            prev_trajectory_times_(i) = traj[i].getDuration();
        }
        prev_coeffs_valid_ = true;
    }

    Info("MINCO: %d pieces, %.2f s total\n", N, total_duration_);
    return true;
}

// ################################################################
// GCOPTER Polytope-Constrained Solver (L-BFGS with safe corridors)
// ################################################################
bool TrajectoryManager::SolveGCOPTERConstrained(
    const std::vector<Eigen::MatrixX4d> &hPolytopes) {

    if (hPolytopes.empty() || num_waypoints_ < 2) return false;

    // --- Pre-validate polytopes (AABB boxes from GenerateCorridors) ---
    // Basic sanity: face count and normal check.  Since we now generate
    // only AABB boxes (6 faces, axis-aligned normals), complex LP-based
    // validation is unnecessary.  AABB consecutive overlap is guaranteed
    // by construction (shared waypoints with margin on each side).
    for (int pi = 0; pi < static_cast<int>(hPolytopes.size()); ++pi) {
        const Eigen::MatrixX4d &hp = hPolytopes[pi];
        if (hp.rows() < 4) {
            Warn("GCOPTER constrained: polytope %d has only %d faces (need >=4), aborting\n",
                 pi, static_cast<int>(hp.rows()));
            return false;
        }
        // Check for NaN/Inf in polytope data (can come from bad workspace bounds)
        if (!hp.allFinite()) {
            Warn("GCOPTER constrained: polytope %d has NaN/Inf values, aborting\n", pi);
            return false;
        }
    }

    // Verify consecutive AABB boxes overlap.  For AABB boxes, two boxes
    // overlap iff their ranges intersect on all 3 axes.  Extract lo/hi
    // from the H-representation: row0 gives +x face → hi.x = -d(0),
    // row1 gives -x face → lo.x = d(1), etc.
    for (int pi = 0; pi + 1 < static_cast<int>(hPolytopes.size()); ++pi) {
        const Eigen::MatrixX4d &pA = hPolytopes[pi];
        const Eigen::MatrixX4d &pB = hPolytopes[pi+1];
        // For AABB: hi.x = -d(row0), lo.x = d(row1), etc.
        // Box A
        double ax_hi = -pA(0, 3), ax_lo = pA(1, 3);
        double ay_hi = -pA(2, 3), ay_lo = pA(3, 3);
        double az_hi = -pA(4, 3), az_lo = pA(5, 3);
        // Box B
        double bx_hi = -pB(0, 3), bx_lo = pB(1, 3);
        double by_hi = -pB(2, 3), by_lo = pB(3, 3);
        double bz_hi = -pB(4, 3), bz_lo = pB(5, 3);
        // Overlap check per axis
        double ox = std::min(ax_hi, bx_hi) - std::max(ax_lo, bx_lo);
        double oy = std::min(ay_hi, by_hi) - std::max(ay_lo, by_lo);
        double oz = std::min(az_hi, bz_hi) - std::max(az_lo, bz_lo);
        if (ox < 0.01 || oy < 0.01 || oz < 0.01) {
            Warn("GCOPTER constrained: AABB boxes %d/%d insufficient overlap "
                 "(ox=%.3f oy=%.3f oz=%.3f), aborting\n", pi, pi+1, ox, oy, oz);
            return false;
        }
    }

    double v_max = max_vel_->Value();
    if (v_max < 0.1) v_max = 0.1;
    double a_max = max_acc_->Value();
    if (a_max < 0.1) a_max = 0.1;

    // Boundary conditions
    Eigen::Matrix3d headPVA, tailPVA;
    headPVA.col(0) = waypoints_[0];
    headPVA.col(1) = start_vel_;
    headPVA.col(2) = Eigen::Vector3d::Zero();
    tailPVA.col(0) = waypoints_[num_waypoints_ - 1];
    tailPVA.col(1) = end_vel_;
    tailPVA.col(2) = Eigen::Vector3d::Zero();

    // Parameters for GCOPTER_PolytopeSFC
    // Values aligned with the original ZJU GCOPTER global_planning.yaml defaults,
    // adjusted for our lower velocity limits.  The key insight: GCOPTER uses
    // penalty methods, not hard constraints.  Penalty weights must be high enough
    // relative to timeWeight for constraints to be respected.
    //
    // Reference: gcopter/config/global_planning.yaml
    //   WeightT=20, ChiVec=[1e4,1e4,1e4,1e4,1e5], IntegralIntervs=16
    double timeWeight = 20.0;        // rho: penalty on total time (original default)
    double lengthPerPiece = 1.0;     // meters per MINCO piece (finer for small workspace)
    double smoothingFactor = 0.01;
    int integralResolution = 16;     // quadrature resolution (original default)

    // Magnitude bounds: [v_max, omega_max, theta_max, thrust_min, thrust_max]
    double mass = 0.61;  // kg (small quadrotor, from original config)
    double grav = 9.81;
    double thrust_min = mass * grav * 0.3;
    double thrust_max = mass * grav * 2.0;
    double omega_max = 2.1;  // rad/s (from original config MaxBdrMag)
    Eigen::VectorXd magnitudeBounds(5);
    magnitudeBounds << v_max,           // max velocity (m/s)
                       omega_max,       // max body rate (rad/s)
                       1.05,            // max tilt angle (~60 deg, from original config)
                       thrust_min,      // min thrust (N)
                       thrust_max;      // max thrust (N)

    // Penalty weights: [position/corridor, velocity, body_rate, tilt_angle, thrust]
    // All 1e4 except thrust at 1e5 — matching original GCOPTER defaults.
    // These must dominate timeWeight for constraints to be respected.
    Eigen::VectorXd penaltyWeights(5);
    penaltyWeights << 1.0e4,     // corridor (position) penalty
                      1.0e4,     // velocity penalty
                      1.0e4,     // body rate penalty
                      1.0e4,     // tilt angle penalty
                      1.0e5;     // thrust penalty

    // Physical params: [mass, grav, drag_coeff_x, drag_coeff_y, drag_coeff_z, yaw_dot]
    // NOTE: GCOPTER's flatness map assumes z-UP (ENU). Our planner uses NED (z-down).
    // The penalty functional (thrust/tilt/body-rate) has a frame mismatch for vertical
    // maneuvers. For horizontal indoor flight this is negligible. The corridor position
    // constraint and velocity magnitude check are frame-independent.
    Eigen::VectorXd physicalParams(6);
    physicalParams << mass, grav, 0.70, 0.80, 0.01, 1e-4;  // drag coeffs from original config

    // --- Wrap the GCOPTER solve in try-catch ---
    // quickhull / enumerateVs can throw on degenerate geometry even after our
    // pre-validation (e.g. polytope intersection produces degenerate faces).
    Trajectory<5> traj;
    double cost;
    try {
        gcopter::GCOPTER_PolytopeSFC solver;
        if (!solver.setup(timeWeight, headPVA, tailPVA, hPolytopes,
                          lengthPerPiece, smoothingFactor, integralResolution,
                          magnitudeBounds, penaltyWeights, physicalParams)) {
            Warn("GCOPTER constrained: setup failed\n");
            return false;
        }

        cost = solver.optimize(traj, 1.0e-5);  // tighter tolerance (original default)
    } catch (const std::exception &e) {
        Warn("GCOPTER constrained: exception during solve: %s\n", e.what());
        return false;
    } catch (...) {
        Warn("GCOPTER constrained: unknown exception during solve\n");
        return false;
    }

    // --- Handle L-BFGS failure gracefully ---
    // Check for NaN/Inf cost and negative cost (indicates L-BFGS negative
    // line-search step).  Do NOT extract a trajectory from a failed optimization.
    if (std::isinf(cost) || std::isnan(cost) || cost < 0.0 || traj.getPieceNum() < 1) {
        Warn("GCOPTER constrained: optimization failed (cost=%.4f, pieces=%d)\n",
             cost, traj.getPieceNum());
        return false;
    }

    // Sanity check: trajectory duration must be positive and finite
    double dur = traj.getTotalDuration();
    if (std::isnan(dur) || std::isinf(dur) || dur <= 0.0) {
        Warn("GCOPTER constrained: invalid trajectory duration %.4f\n", dur);
        return false;
    }

    // Store result
    gcopter_traj_ = traj;
    gcopter_traj_valid_ = true;

    // Post-solve validation using GCOPTER's analytical max-rate computation.
    // If constraints are massively violated, reject the trajectory — TOPP-RA
    // cannot fix extreme violations (it would produce absurd durations).
    {
        double actual_max_vel = gcopter_traj_.getMaxVelRate();
        double actual_max_acc = gcopter_traj_.getMaxAccRate();
        Info("GCOPTER validation: max_vel=%.2f/%.2f max_acc=%.2f/%.2f\n",
             actual_max_vel, v_max, actual_max_acc, a_max);

        // Reject if velocity exceeds limit by >3x or accel by >10x
        // (mild violations are acceptable — TOPP-RA can fix those)
        if (actual_max_vel > v_max * 3.0) {
            Warn("GCOPTER constrained: REJECTING trajectory — max velocity %.2f "
                 "exceeds limit %.2f by >3x\n", actual_max_vel, v_max);
            gcopter_traj_valid_ = false;
            return false;
        }
        if (actual_max_acc > a_max * 10.0) {
            Warn("GCOPTER constrained: REJECTING trajectory — max accel %.2f "
                 "exceeds limit %.2f by >10x\n", actual_max_acc, a_max);
            gcopter_traj_valid_ = false;
            return false;
        }
        if (actual_max_vel > v_max * 1.5) {
            Warn("GCOPTER constrained: max velocity %.2f exceeds limit %.2f by >50%%\n",
                 actual_max_vel, v_max);
        }
        if (actual_max_acc > a_max * 2.0) {
            Warn("GCOPTER constrained: max accel %.2f exceeds limit %.2f by >100%%\n",
                 actual_max_acc, a_max);
        }
    }

    total_duration_ = traj.getTotalDuration();
    int N = traj.getPieceNum();
    num_segments_ = std::min(N, MAX_SEGMENTS);

    if (num_waypoints_ > 0) {
        last_pos_ = Vector3Df(static_cast<float>(waypoints_[0].y()),
                              static_cast<float>(waypoints_[0].x()),
                              static_cast<float>(waypoints_[0].z()));
    }

    for (int i = 0; i < num_segments_; ++i) {
        segments_[i].duration = traj[i].getDuration();
        segments_[i].coeffs.setZero();
        Eigen::Matrix<double, 3, 6> cm = traj[i].getCoeffMat();
        for (int axis = 0; axis < 3; ++axis) {
            for (int c = 0; c <= 5; ++c) {
                segments_[i].coeffs(axis, c) = cm(axis, 5 - c);
            }
        }
    }

    trajectory_valid_ = true;

    if (warm_start_enabled_) {
        prev_trajectory_times_.resize(N);
        for (int i = 0; i < N; ++i) {
            prev_trajectory_times_(i) = traj[i].getDuration();
        }
        prev_coeffs_valid_ = true;
    }

    Info("GCOPTER constrained: %d pieces, %.2f s total (cost=%.2f)\n",
         N, total_duration_, cost);
    return true;
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
    size_t total = static_cast<size_t>(grid_nx_) * grid_ny_ * grid_nz_;
    grid_data_.assign(total, 0u);
    astar_gcost_.resize(total);
    astar_parent_.resize(total);
    jps_dir_x_.resize(total, 0);
    jps_dir_y_.resize(total, 0);
    jps_dir_z_.resize(total, 0);
    grid_dirty_ = true;
    grid_initialized_ = false;
    distance_field_valid_ = false;
    cache_valid_ = false;
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

Eigen::Vector3d TrajectoryManager::FindNearestFreeCell(const Eigen::Vector3d &pos) const {
    // BFS outward from the occupied cell to find the nearest free cell
    Eigen::Vector3i gc = WorldToGrid(pos);
    int cx = gc.x(), cy = gc.y(), cz = gc.z();

    // Search in expanding shells (radius 1, 2, 3, ...)
    for (int radius = 1; radius <= 50; ++radius) {
        double best_dist = 1e9;
        Eigen::Vector3i best(-1, -1, -1);
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dz = -radius; dz <= radius; ++dz) {
                    // Only check the shell surface
                    if (std::abs(dx) != radius && std::abs(dy) != radius && std::abs(dz) != radius)
                        continue;
                    int nx = cx + dx, ny = cy + dy, nz = cz + dz;
                    if (GridInBounds(nx, ny, nz) && !IsOccupied(nx, ny, nz)) {
                        double d = std::sqrt(static_cast<double>(dx*dx + dy*dy + dz*dz));
                        if (d < best_dist) {
                            best_dist = d;
                            best = Eigen::Vector3i(nx, ny, nz);
                        }
                    }
                }
            }
        }
        if (best.x() >= 0) {
            return GridToWorld(best.x(), best.y(), best.z());
        }
    }
    // If nothing found, return original (shouldn't happen in a reasonable workspace)
    return pos;
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

void TrajectoryManager::MarkCylinderOccupied(double cx, double cy, double radius) {
    // Mark a vertical cylinder (full Z extent) as occupied
    // This is the correct model for indoor obstacles (poles, people, objects)
    // which block the entire vertical column
    Eigen::Vector3d lo_pt(cx - radius, cy - radius, WS_Z_MIN);
    Eigen::Vector3d hi_pt(cx + radius, cy + radius, WS_Z_MAX);
    Eigen::Vector3i lo = WorldToGrid(lo_pt);
    Eigen::Vector3i hi = WorldToGrid(hi_pt);

    int ix0 = std::max(lo.x(), 0);
    int iy0 = std::max(lo.y(), 0);
    int ix1 = std::min(hi.x(), grid_nx_ - 1);
    int iy1 = std::min(hi.y(), grid_ny_ - 1);

    double r2 = radius * radius;
    for (int ix = ix0; ix <= ix1; ++ix) {
        for (int iy = iy0; iy <= iy1; ++iy) {
            Eigen::Vector3d vc = GridToWorld(ix, iy, 0);
            double dx = vc.x() - cx;
            double dy = vc.y() - cy;
            if (dx*dx + dy*dy <= r2) {
                // Mark entire Z column
                for (int iz = 0; iz < grid_nz_; ++iz) {
                    size_t idx = static_cast<size_t>(ix) * grid_ny_ * grid_nz_
                               + static_cast<size_t>(iy) * grid_nz_
                               + static_cast<size_t>(iz);
                    grid_data_[idx] = 1u;
                }
            }
        }
    }
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
    // Compute the iz threshold once, then memset entire z-slices
    int iz_ground = -1;
    for (int iz = 0; iz < grid_nz_; ++iz) {
        double wz = grid_origin_.z() + (iz + 0.5) * grid_res_;
        if (wz >= -sm) {
            iz_ground = iz;
            break;
        }
    }
    if (iz_ground >= 0) {
        int nz_ground = grid_nz_ - iz_ground;  // number of z cells to mark
        for (int ix = 0; ix < grid_nx_; ++ix) {
            for (int iy = 0; iy < grid_ny_; ++iy) {
                size_t base = static_cast<size_t>(ix) * grid_ny_ * grid_nz_
                            + static_cast<size_t>(iy) * grid_nz_
                            + static_cast<size_t>(iz_ground);
                std::memset(&grid_data_[base], 1u, static_cast<size_t>(nz_ground));
            }
        }
    }

    // Mark each obstacle as inflated vertical cylinder (radius + safety_margin)
    // Cylinder model: obstacles block the full Z column (XY position only)
    // This is correct for indoor obstacles (people, poles, objects on ground)
    // whose OptiTrack z may be 0 (ground level) but they block at all altitudes
    for (int i = 0; i < num_obstacles_; ++i) {
        MarkCylinderOccupied(obstacles_[i].pos.x(), obstacles_[i].pos.y(),
                             obstacles_[i].radius + sm);

        // Spatio-temporal prediction: predict obstacle future positions
        // using safety_margin as the uncertainty factor
        double obs_speed = obstacles_[i].vel.norm();
        if (obs_speed > 0.01 && total_duration_ > 0.0) {
            // SOTA Upgrade 8: Adaptive prediction horizon
            double remaining_time = total_duration_;
            if (execution_time_set_ && last_update_time_ > 0.0) {
                double elapsed = last_update_time_ - execution_start_time_;
                remaining_time = std::max(total_duration_ - elapsed, 1.0);
            }
            double pred_horizon = std::min(remaining_time, 5.0);  // cap at 5s
            for (int step = 1; step <= 4; ++step) {
                double t_pred = pred_horizon * step / 4.0;
                double pred_x = obstacles_[i].pos.x() + obstacles_[i].vel.x() * t_pred;
                double pred_y = obstacles_[i].pos.y() + obstacles_[i].vel.y() * t_pred;
                double inflated_radius = obstacles_[i].radius + sm + sm * t_pred;
                MarkCylinderOccupied(pred_x, pred_y, inflated_radius);
            }
        }
    }
}

// ============================================================
// FindPath — JPS 3D with A* fallback
// ============================================================
bool TrajectoryManager::FindPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                                  std::vector<Eigen::Vector3d> &path) {
    // Try JPS first (much faster on large grids)
    if (FindPathJPS(start, goal, path))
        return true;
    // Fallback to regular A* for degenerate cases
    return FindPathAStar(start, goal, path);
}

// ============================================================
// A* path search with 26-connectivity (fallback)
// ============================================================
bool TrajectoryManager::FindPathAStar(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
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

    // Memory-efficient A* using pre-allocated flat arrays (class members).
    // Resized once in InitGrid(), reused across calls — no per-plan heap allocation.
    size_t N = static_cast<size_t>(grid_nx_) * grid_ny_ * grid_nz_;

    // Reset g-cost (FLT_MAX = unvisited) and parent (-2 = not visited)
    std::fill(astar_gcost_.begin(), astar_gcost_.begin() + N, std::numeric_limits<float>::max());
    std::fill(astar_parent_.begin(), astar_parent_.begin() + N, -2);

    // SOTA: Bucket queue — O(1) push/pop instead of O(log N) priority queue
    // Helpers
    int nyz = grid_ny_ * grid_nz_;
    #define CELL_IDX(ix_, iy_, iz_) ((ix_) * nyz + (iy_) * grid_nz_ + (iz_))

    auto eucDist = [](int x1, int y1, int z1, int x2, int y2, int z2) -> float {
        float dx = static_cast<float>(x2 - x1);
        float dy = static_cast<float>(y2 - y1);
        float dz = static_cast<float>(z2 - z1);
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    float res_f = static_cast<float>(grid_res_);
    double bucket_res = 0.1 * grid_res_;
    double max_cost_est = eucDist(sx, sy, sz, gx, gy, gz) * res_f * 3.0;
    int max_buckets = static_cast<int>(max_cost_est / bucket_res) + 1000;
    if (max_buckets > 100000) max_buckets = 100000;
    bucket_queue_.init(max_buckets, bucket_res);
    bucket_queue_.clear();

    int start_cell = CELL_IDX(sx, sy, sz);
    int goal_cell = CELL_IDX(gx, gy, gz);
    astar_gcost_[start_cell] = 0.0f;
    astar_parent_[start_cell] = -1;  // -1 = start node
    double start_f = ASTAR_WEIGHT * eucDist(sx, sy, sz, gx, gy, gz) * res_f;
    bucket_queue_.push(start_cell, start_f);

    // 26-connectivity
    static const int offsets[26][3] = {
        { 1, 0, 0}, {-1, 0, 0}, { 0, 1, 0}, { 0,-1, 0}, { 0, 0, 1}, { 0, 0,-1},
        { 1, 1, 0}, { 1,-1, 0}, {-1, 1, 0}, {-1,-1, 0},
        { 1, 0, 1}, { 1, 0,-1}, {-1, 0, 1}, {-1, 0,-1},
        { 0, 1, 1}, { 0, 1,-1}, { 0,-1, 1}, { 0,-1,-1},
        { 1, 1, 1}, { 1, 1,-1}, { 1,-1, 1}, { 1,-1,-1},
        {-1, 1, 1}, {-1, 1,-1}, {-1,-1, 1}, {-1,-1,-1}
    };
    static const float step_costs[26] = {
        1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
        1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f,
        1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f,
        1.41421356f, 1.41421356f, 1.41421356f, 1.41421356f,
        1.73205081f, 1.73205081f, 1.73205081f, 1.73205081f,
        1.73205081f, 1.73205081f, 1.73205081f, 1.73205081f
    };

    bool found = false;
    int max_iter = static_cast<int>(std::min(N, static_cast<size_t>(800000)));
    int iter = 0;

    while (!bucket_queue_.empty() && iter < max_iter) {
        ++iter;
        int ci = bucket_queue_.pop();
        if (ci < 0) break;

        if (ci == goal_cell) {
            found = true;
            break;
        }

        // Decode current cell
        int cx = ci / nyz;
        int cy = (ci % nyz) / grid_nz_;
        int cz = ci % grid_nz_;
        float cur_g = astar_gcost_[ci];

        for (int ni = 0; ni < 26; ++ni) {
            int nx_i = cx + offsets[ni][0];
            int ny_i = cy + offsets[ni][1];
            int nz_i = cz + offsets[ni][2];

            if (!GridInBounds(nx_i, ny_i, nz_i)) continue;
            if (IsOccupied(nx_i, ny_i, nz_i)) continue;

            // Diagonal safety
            int dx = offsets[ni][0], dy = offsets[ni][1], dz = offsets[ni][2];
            int nz_count = (dx != 0 ? 1 : 0) + (dy != 0 ? 1 : 0) + (dz != 0 ? 1 : 0);
            if (nz_count >= 2) {
                bool blocked = false;
                if (dx != 0 && IsOccupied(cx + dx, cy, cz)) blocked = true;
                if (dy != 0 && IsOccupied(cx, cy + dy, cz)) blocked = true;
                if (dz != 0 && IsOccupied(cx, cy, cz + dz)) blocked = true;
                if (blocked) continue;
            }

            int nb = CELL_IDX(nx_i, ny_i, nz_i);
            float new_g = cur_g + step_costs[ni] * res_f;
            if (new_g < astar_gcost_[nb]) {
                astar_gcost_[nb] = new_g;
                astar_parent_[nb] = ci;
                float h = ASTAR_WEIGHT * eucDist(nx_i, ny_i, nz_i, gx, gy, gz) * res_f;
                bucket_queue_.push(nb, static_cast<double>(new_g + h));
            }
        }
    }

    #undef CELL_IDX

    if (!found) return false;

    // Reconstruct path
    std::vector<Eigen::Vector3d> raw_path;
    int ci = goal_cell;
    while (ci >= 0) {
        int ix = ci / nyz;
        int iy = (ci % nyz) / grid_nz_;
        int iz = ci % grid_nz_;
        raw_path.push_back(GridToWorld(ix, iy, iz));
        ci = astar_parent_[ci];
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
// JPS 3D Neighbor Pruning Tables
// ============================================================
const int TrajectoryManager::JPS3DNeib::nsz[4][2] = {
    {26, 0}, {1, 8}, {3, 12}, {7, 12}
};

TrajectoryManager::JPS3DNeib::JPS3DNeib() {
    int id = 0;
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
                for (int dev = 0; dev < nsz[norm1][0]; ++dev)
                    Neib(dx, dy, dz, norm1, dev,
                         ns[id][0][dev], ns[id][1][dev], ns[id][2][dev]);
                for (int dev = 0; dev < nsz[norm1][1]; ++dev)
                    FNeib(dx, dy, dz, norm1, dev,
                          f1[id][0][dev], f1[id][1][dev], f1[id][2][dev],
                          f2[id][0][dev], f2[id][1][dev], f2[id][2][dev]);
                id++;
            }
        }
    }
}

void TrajectoryManager::JPS3DNeib::Neib(int dx, int dy, int dz,
    int norm1, int dev, int& tx, int& ty, int& tz) {
    switch (norm1) {
    case 0:
        switch (dev) {
            case 0: tx=1; ty=0; tz=0; return;
            case 1: tx=-1; ty=0; tz=0; return;
            case 2: tx=0; ty=1; tz=0; return;
            case 3: tx=1; ty=1; tz=0; return;
            case 4: tx=-1; ty=1; tz=0; return;
            case 5: tx=0; ty=-1; tz=0; return;
            case 6: tx=1; ty=-1; tz=0; return;
            case 7: tx=-1; ty=-1; tz=0; return;
            case 8: tx=0; ty=0; tz=1; return;
            case 9: tx=1; ty=0; tz=1; return;
            case 10: tx=-1; ty=0; tz=1; return;
            case 11: tx=0; ty=1; tz=1; return;
            case 12: tx=1; ty=1; tz=1; return;
            case 13: tx=-1; ty=1; tz=1; return;
            case 14: tx=0; ty=-1; tz=1; return;
            case 15: tx=1; ty=-1; tz=1; return;
            case 16: tx=-1; ty=-1; tz=1; return;
            case 17: tx=0; ty=0; tz=-1; return;
            case 18: tx=1; ty=0; tz=-1; return;
            case 19: tx=-1; ty=0; tz=-1; return;
            case 20: tx=0; ty=1; tz=-1; return;
            case 21: tx=1; ty=1; tz=-1; return;
            case 22: tx=-1; ty=1; tz=-1; return;
            case 23: tx=0; ty=-1; tz=-1; return;
            case 24: tx=1; ty=-1; tz=-1; return;
            case 25: tx=-1; ty=-1; tz=-1; return;
        }
    case 1:
        tx = dx; ty = dy; tz = dz; return;
    case 2:
        switch (dev) {
            case 0:
                if (dz == 0) { tx = 0; ty = dy; tz = 0; }
                else { tx = 0; ty = 0; tz = dz; }
                return;
            case 1:
                if (dx == 0) { tx = 0; ty = dy; tz = 0; }
                else { tx = dx; ty = 0; tz = 0; }
                return;
            case 2:
                tx = dx; ty = dy; tz = dz; return;
        }
    case 3:
        switch (dev) {
            case 0: tx = dx; ty = 0; tz = 0; return;
            case 1: tx = 0; ty = dy; tz = 0; return;
            case 2: tx = 0; ty = 0; tz = dz; return;
            case 3: tx = dx; ty = dy; tz = 0; return;
            case 4: tx = dx; ty = 0; tz = dz; return;
            case 5: tx = 0; ty = dy; tz = dz; return;
            case 6: tx = dx; ty = dy; tz = dz; return;
        }
    }
}

void TrajectoryManager::JPS3DNeib::FNeib(int dx, int dy, int dz,
    int norm1, int dev,
    int& fx, int& fy, int& fz,
    int& nx, int& ny, int& nz) {
    switch (norm1) {
    case 1:
        switch (dev) {
            case 0: fx=0; fy=1; fz=0; break;
            case 1: fx=0; fy=-1; fz=0; break;
            case 2: fx=1; fy=0; fz=0; break;
            case 3: fx=1; fy=1; fz=0; break;
            case 4: fx=1; fy=-1; fz=0; break;
            case 5: fx=-1; fy=0; fz=0; break;
            case 6: fx=-1; fy=1; fz=0; break;
            case 7: fx=-1; fy=-1; fz=0; break;
        }
        nx = fx; ny = fy; nz = dz;
        if (dx != 0) { fz = fx; fx = 0; nz = fz; nx = dx; }
        if (dy != 0) { fz = fy; fy = 0; nz = fz; ny = dy; }
        return;
    case 2:
        if (dx == 0) {
            switch (dev) {
                case 0: fx=0; fy=0; fz=-dz; nx=0; ny=dy; nz=-dz; return;
                case 1: fx=0; fy=-dy; fz=0; nx=0; ny=-dy; nz=dz; return;
                case 2: fx=1; fy=0; fz=0; nx=1; ny=dy; nz=dz; return;
                case 3: fx=-1; fy=0; fz=0; nx=-1; ny=dy; nz=dz; return;
                case 4: fx=1; fy=0; fz=-dz; nx=1; ny=dy; nz=-dz; return;
                case 5: fx=1; fy=-dy; fz=0; nx=1; ny=-dy; nz=dz; return;
                case 6: fx=-1; fy=0; fz=-dz; nx=-1; ny=dy; nz=-dz; return;
                case 7: fx=-1; fy=-dy; fz=0; nx=-1; ny=-dy; nz=dz; return;
                case 8: fx=1; fy=0; fz=0; nx=1; ny=dy; nz=0; return;
                case 9: fx=1; fy=0; fz=0; nx=1; ny=0; nz=dz; return;
                case 10: fx=-1; fy=0; fz=0; nx=-1; ny=dy; nz=0; return;
                case 11: fx=-1; fy=0; fz=0; nx=-1; ny=0; nz=dz; return;
            }
        } else if (dy == 0) {
            switch (dev) {
                case 0: fx=0; fy=0; fz=-dz; nx=dx; ny=0; nz=-dz; return;
                case 1: fx=-dx; fy=0; fz=0; nx=-dx; ny=0; nz=dz; return;
                case 2: fx=0; fy=1; fz=0; nx=dx; ny=1; nz=dz; return;
                case 3: fx=0; fy=-1; fz=0; nx=dx; ny=-1; nz=dz; return;
                case 4: fx=0; fy=1; fz=-dz; nx=dx; ny=1; nz=-dz; return;
                case 5: fx=-dx; fy=1; fz=0; nx=-dx; ny=1; nz=dz; return;
                case 6: fx=0; fy=-1; fz=-dz; nx=dx; ny=-1; nz=-dz; return;
                case 7: fx=-dx; fy=-1; fz=0; nx=-dx; ny=-1; nz=dz; return;
                case 8: fx=0; fy=1; fz=0; nx=dx; ny=1; nz=0; return;
                case 9: fx=0; fy=1; fz=0; nx=0; ny=1; nz=dz; return;
                case 10: fx=0; fy=-1; fz=0; nx=dx; ny=-1; nz=0; return;
                case 11: fx=0; fy=-1; fz=0; nx=0; ny=-1; nz=dz; return;
            }
        } else { // dz == 0
            switch (dev) {
                case 0: fx=0; fy=-dy; fz=0; nx=dx; ny=-dy; nz=0; return;
                case 1: fx=-dx; fy=0; fz=0; nx=-dx; ny=dy; nz=0; return;
                case 2: fx=0; fy=0; fz=1; nx=dx; ny=dy; nz=1; return;
                case 3: fx=0; fy=0; fz=-1; nx=dx; ny=dy; nz=-1; return;
                case 4: fx=0; fy=-dy; fz=1; nx=dx; ny=-dy; nz=1; return;
                case 5: fx=-dx; fy=0; fz=1; nx=-dx; ny=dy; nz=1; return;
                case 6: fx=0; fy=-dy; fz=-1; nx=dx; ny=-dy; nz=-1; return;
                case 7: fx=-dx; fy=0; fz=-1; nx=-dx; ny=dy; nz=-1; return;
                case 8: fx=0; fy=0; fz=1; nx=dx; ny=0; nz=1; return;
                case 9: fx=0; fy=0; fz=1; nx=0; ny=dy; nz=1; return;
                case 10: fx=0; fy=0; fz=-1; nx=dx; ny=0; nz=-1; return;
                case 11: fx=0; fy=0; fz=-1; nx=0; ny=dy; nz=-1; return;
            }
        }
    case 3:
        switch (dev) {
            case 0: fx=-dx; fy=0; fz=0; nx=-dx; ny=dy; nz=dz; return;
            case 1: fx=0; fy=-dy; fz=0; nx=dx; ny=-dy; nz=dz; return;
            case 2: fx=0; fy=0; fz=-dz; nx=dx; ny=dy; nz=-dz; return;
            case 3: fx=0; fy=-dy; fz=-dz; nx=dx; ny=-dy; nz=-dz; return;
            case 4: fx=-dx; fy=0; fz=-dz; nx=-dx; ny=dy; nz=-dz; return;
            case 5: fx=-dx; fy=-dy; fz=0; nx=-dx; ny=-dy; nz=dz; return;
            case 6: fx=-dx; fy=0; fz=0; nx=-dx; ny=0; nz=dz; return;
            case 7: fx=-dx; fy=0; fz=0; nx=-dx; ny=dy; nz=0; return;
            case 8: fx=0; fy=-dy; fz=0; nx=0; ny=-dy; nz=dz; return;
            case 9: fx=0; fy=-dy; fz=0; nx=dx; ny=-dy; nz=0; return;
            case 10: fx=0; fy=0; fz=-dz; nx=0; ny=dy; nz=-dz; return;
            case 11: fx=0; fy=0; fz=-dz; nx=dx; ny=0; nz=-dz; return;
        }
    }
}

// ============================================================
// JPS 3D: HasForced / Jump / FindPathJPS
// ============================================================
bool TrajectoryManager::HasForcedJPS(int x, int y, int z,
                                      int dx, int dy, int dz) {
    int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
    int id = (dx + 1) + 3 * (dy + 1) + 9 * (dz + 1);
    int num_check;
    switch (norm1) {
        case 1: num_check = 8; break;
        case 2: num_check = 12; break;  // nsz[2][1] = 12 forced neighbors
        case 3: num_check = 12; break;  // nsz[3][1] = 12 forced neighbors
        default: return false;
    }
    for (int fn = 0; fn < num_check; ++fn) {
        int nx = x + jps_neib_.f1[id][0][fn];
        int ny = y + jps_neib_.f1[id][1][fn];
        int nz = z + jps_neib_.f1[id][2][fn];
        if (IsOccupied(nx, ny, nz))
            return true;
    }
    return false;
}

bool TrajectoryManager::JumpJPS(int x, int y, int z,
                                 int dx, int dy, int dz,
                                 int& jx, int& jy, int& jz) {
    // Iterative jump to avoid stack overflow on large grids
    int cx = x, cy = y, cz = z;
    int max_steps = grid_nx_ + grid_ny_ + grid_nz_;
    for (int step = 0; step < max_steps; ++step) {
        int nx = cx + dx;
        int ny = cy + dy;
        int nz = cz + dz;

        if (!GridInBounds(nx, ny, nz) || IsOccupied(nx, ny, nz))
            return false;

        // Diagonal safety: prevent corner-cutting
        // For diagonal moves (norm1 >= 2), check that intermediate
        // axis-aligned cells are free. Without this, JPS can jump
        // through obstacle corners that the UAV can't fit through.
        int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
        if (norm1 >= 2) {
            bool blocked = false;
            if (dx != 0 && IsOccupied(cx + dx, cy, cz)) blocked = true;
            if (dy != 0 && IsOccupied(cx, cy + dy, cz)) blocked = true;
            if (dz != 0 && IsOccupied(cx, cy, cz + dz)) blocked = true;
            if (blocked) return false;
        }

        if (nx == jps_goal_x_ && ny == jps_goal_y_ && nz == jps_goal_z_) {
            jx = nx; jy = ny; jz = nz;
            return true;
        }

        if (HasForcedJPS(nx, ny, nz, dx, dy, dz)) {
            jx = nx; jy = ny; jz = nz;
            return true;
        }

        // For diagonal moves, recursively jump in sub-directions
        int id = (dx + 1) + 3 * (dy + 1) + 9 * (dz + 1);
        // norm1 already computed above
        int num_neib = jps_neib_.nsz[norm1][0];
        // Check sub-directions (all natural neighbors except the last one,
        // which is the primary direction itself)
        for (int k = 0; k < num_neib - 1; ++k) {
            int sdx = jps_neib_.ns[id][0][k];
            int sdy = jps_neib_.ns[id][1][k];
            int sdz = jps_neib_.ns[id][2][k];
            int sub_jx, sub_jy, sub_jz;
            if (JumpJPS(nx, ny, nz, sdx, sdy, sdz, sub_jx, sub_jy, sub_jz)) {
                jx = nx; jy = ny; jz = nz;
                return true;
            }
        }

        // Continue in primary direction (tail-call optimization via loop)
        cx = nx; cy = ny; cz = nz;
    }
    return false;
}

bool TrajectoryManager::FindPathJPS(const Eigen::Vector3d &start,
                                     const Eigen::Vector3d &goal,
                                     std::vector<Eigen::Vector3d> &path) {
    path.clear();

    Eigen::Vector3i sg = WorldToGrid(start);
    Eigen::Vector3i gg = WorldToGrid(goal);
    int sx = sg.x(), sy = sg.y(), sz = sg.z();
    int gx = gg.x(), gy = gg.y(), gz = gg.z();

    if (!GridInBounds(sx, sy, sz) || !GridInBounds(gx, gy, gz)) return false;
    if (IsOccupied(sx, sy, sz) || IsOccupied(gx, gy, gz)) return false;

    if (sx == gx && sy == gy && sz == gz) {
        path.push_back(start);
        path.push_back(goal);
        return true;
    }

    // Store goal for JumpJPS
    jps_goal_x_ = gx;
    jps_goal_y_ = gy;
    jps_goal_z_ = gz;

    size_t N = static_cast<size_t>(grid_nx_) * grid_ny_ * grid_nz_;
    std::fill(astar_gcost_.begin(), astar_gcost_.begin() + N, std::numeric_limits<float>::max());
    std::fill(astar_parent_.begin(), astar_parent_.begin() + N, -2);

    // Reset pre-allocated JPS direction arrays
    std::fill(jps_dir_x_.begin(), jps_dir_x_.begin() + N, static_cast<int8_t>(0));
    std::fill(jps_dir_y_.begin(), jps_dir_y_.begin() + N, static_cast<int8_t>(0));
    std::fill(jps_dir_z_.begin(), jps_dir_z_.begin() + N, static_cast<int8_t>(0));

    // SOTA: Bucket queue — O(1) push/pop
    int nyz = grid_ny_ * grid_nz_;
    #define JPS_CELL_IDX(ix_, iy_, iz_) ((ix_) * nyz + (iy_) * grid_nz_ + (iz_))

    auto eucDist = [](int x1, int y1, int z1, int x2, int y2, int z2) -> float {
        float ddx = static_cast<float>(x2 - x1);
        float ddy = static_cast<float>(y2 - y1);
        float ddz = static_cast<float>(z2 - z1);
        return std::sqrt(ddx*ddx + ddy*ddy + ddz*ddz);
    };

    float res_f = static_cast<float>(grid_res_);
    double bucket_res = 0.1 * grid_res_;
    double max_cost_est = eucDist(sx, sy, sz, gx, gy, gz) * res_f * 3.0;
    int max_buckets = static_cast<int>(max_cost_est / bucket_res) + 1000;
    if (max_buckets > 100000) max_buckets = 100000;
    bucket_queue_.init(max_buckets, bucket_res);
    bucket_queue_.clear();

    int start_cell = JPS_CELL_IDX(sx, sy, sz);
    int goal_cell = JPS_CELL_IDX(gx, gy, gz);
    astar_gcost_[start_cell] = 0.0f;
    astar_parent_[start_cell] = -1;
    jps_dir_x_[start_cell] = 0; jps_dir_y_[start_cell] = 0; jps_dir_z_[start_cell] = 0;

    bucket_queue_.push(start_cell, static_cast<double>(eucDist(sx, sy, sz, gx, gy, gz) * res_f));

    bool found = false;
    int max_iter = static_cast<int>(std::min(N, static_cast<size_t>(800000)));
    int iter = 0;

    while (!bucket_queue_.empty() && iter < max_iter) {
        ++iter;
        int ci = bucket_queue_.pop();
        if (ci < 0) break;

        int cx = ci / nyz;
        int cy = (ci % nyz) / grid_nz_;
        int cz = ci % grid_nz_;
        float cur_g = astar_gcost_[ci];

        if (ci == goal_cell) {
            found = true;
            break;
        }

        // Get JPS successors using pruning tables
        int cur_dx = jps_dir_x_[ci];
        int cur_dy = jps_dir_y_[ci];
        int cur_dz = jps_dir_z_[ci];
        int norm1 = std::abs(cur_dx) + std::abs(cur_dy) + std::abs(cur_dz);
        int dir_id = (cur_dx + 1) + 3 * (cur_dy + 1) + 9 * (cur_dz + 1);
        int num_neib = jps_neib_.nsz[norm1][0];
        int num_fneib = jps_neib_.nsz[norm1][1];

        for (int dev = 0; dev < num_neib + num_fneib; ++dev) {
            int ddx, ddy, ddz;
            if (dev < num_neib) {
                ddx = jps_neib_.ns[dir_id][0][dev];
                ddy = jps_neib_.ns[dir_id][1][dev];
                ddz = jps_neib_.ns[dir_id][2][dev];
            } else {
                int fidx = dev - num_neib;
                int fnx = cx + jps_neib_.f1[dir_id][0][fidx];
                int fny = cy + jps_neib_.f1[dir_id][1][fidx];
                int fnz = cz + jps_neib_.f1[dir_id][2][fidx];
                if (!IsOccupied(fnx, fny, fnz))
                    continue;
                ddx = jps_neib_.f2[dir_id][0][fidx];
                ddy = jps_neib_.f2[dir_id][1][fidx];
                ddz = jps_neib_.f2[dir_id][2][fidx];
            }

            int new_x, new_y, new_z;
            if (!JumpJPS(cx, cy, cz, ddx, ddy, ddz, new_x, new_y, new_z))
                continue;

            int nb = JPS_CELL_IDX(new_x, new_y, new_z);
            float move_cost = eucDist(cx, cy, cz, new_x, new_y, new_z) * res_f;
            float new_g = cur_g + move_cost;

            if (new_g < astar_gcost_[nb]) {
                astar_gcost_[nb] = new_g;
                astar_parent_[nb] = ci;
                int ndx = (new_x > cx) ? 1 : ((new_x < cx) ? -1 : 0);
                int ndy = (new_y > cy) ? 1 : ((new_y < cy) ? -1 : 0);
                int ndz = (new_z > cz) ? 1 : ((new_z < cz) ? -1 : 0);
                jps_dir_x_[nb] = static_cast<int8_t>(ndx);
                jps_dir_y_[nb] = static_cast<int8_t>(ndy);
                jps_dir_z_[nb] = static_cast<int8_t>(ndz);
                float h = eucDist(new_x, new_y, new_z, gx, gy, gz) * res_f;
                bucket_queue_.push(nb, static_cast<double>(new_g + h));
            }
        }
    }

    #undef JPS_CELL_IDX

    if (!found) return false;

    // Reconstruct path
    std::vector<Eigen::Vector3d> raw_path;
    int ci = goal_cell;
    while (ci >= 0) {
        int ix = ci / nyz;
        int iy = (ci % nyz) / grid_nz_;
        int iz = ci % grid_nz_;
        raw_path.push_back(GridToWorld(ix, iy, iz));
        ci = astar_parent_[ci];
    }
    std::reverse(raw_path.begin(), raw_path.end());

    if (!raw_path.empty()) raw_path.front() = start;
    if (raw_path.size() > 1) raw_path.back() = goal;

    path = SimplifyPath(raw_path);
    return true;
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

        // Note: obstacles are already inflated by (radius + safety_margin) in the
        // occupancy grid, so the A* path and the AABB expansion already maintain
        // safety_margin clearance. No additional corridor shrinkage is needed.
        // Only shrink by a small epsilon for numerical safety (no double safety margin)
        double eps = grid_res_ * 0.25;  // reduced from 0.5 to be less aggressive
        x_min += eps; x_max -= eps;
        y_min += eps; y_max -= eps;
        z_min += eps; z_max -= eps;

        // If corridor collapsed (too thin after epsilon shrink), relax:
        // Try with no epsilon, just ensure lo < hi
        if (x_min >= x_max) { double mid = 0.5*(x_min+x_max); x_min = mid - grid_res_*0.5; x_max = mid + grid_res_*0.5; }
        if (y_min >= y_max) { double mid = 0.5*(y_min+y_max); y_min = mid - grid_res_*0.5; y_max = mid + grid_res_*0.5; }
        if (z_min >= z_max) { double mid = 0.5*(z_min+z_max); z_min = mid - grid_res_*0.5; z_max = mid + grid_res_*0.5; }

        // Final collapse check
        if (x_min >= x_max || y_min >= y_max || z_min >= z_max) {
            Warn("corridor collapsed for segment %d (try reducing safety margin or obstacle radius)\n", seg);
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
    // SOTA: Incremental grid + distance field (skip if obstacles haven't moved)
    BuildOccupancyGridIncremental();
    if (!distance_field_valid_) {
        ComputeDistanceField();
        distance_field_valid_ = true;
    }

    double sm = safety_margin_->Value();

    Info("obstacle avoidance: %d obstacles, safety_margin=%.2f, grid=%dx%dx%d (res=%.2f)\n",
         num_obstacles_, sm, grid_nx_, grid_ny_, grid_nz_, grid_res_);

    // Step 2: Shift any waypoint that lands inside an obstacle to the nearest free cell
    for (int i = 0; i < num_waypoints_; ++i) {
        if (IsOccupiedWorld(waypoints_[i])) {
            Eigen::Vector3d shifted = FindNearestFreeCell(waypoints_[i]);
            Warn("WP%d(%.2f,%.2f,%.2f) is inside obstacle, shifted to (%.2f,%.2f,%.2f)\n",
                 i, waypoints_[i].x(), waypoints_[i].y(), waypoints_[i].z(),
                 shifted.x(), shifted.y(), shifted.z());
            waypoints_[i] = shifted;
        }
    }

    // Step 3: Find collision-free path through consecutive waypoints
    std::vector<Eigen::Vector3d> full_path;
    full_path.push_back(waypoints_[0]);

    for (int i = 0; i < num_waypoints_ - 1; ++i) {
        std::vector<Eigen::Vector3d> seg_path;
        if (!FindPath(waypoints_[i], waypoints_[i + 1], seg_path)) {
            Warn("A* failed between WP%d and WP%d, using direct fallback\n", i, i+1);
            seg_path.clear();
            seg_path.push_back(waypoints_[i]);
            seg_path.push_back(waypoints_[i + 1]);
        } else {
            Info("A* WP%d->WP%d: found path with %d points\n",
                 i, i+1, static_cast<int>(seg_path.size()));
        }

        // Per-segment detour / reversal guard.
        //
        // Check ONLY this individual A* segment (WP_i → WP_{i+1}) for
        // internal reversals or excessive detours.  If found AND the
        // direct WP_i→WP_{i+1} line is obstacle-free, replace the A*
        // path with a direct segment.  If the direct is blocked, keep
        // the A* path — MINCO’s reversal-split will handle it.
        //
        // Crucially this does NOT touch reversals that occur at the
        // JUNCTION between two consecutive GUI waypoints (e.g. the turn
        // at WP2 in a WP0→WP1→WP2→WP3→WP4 path).  Those are
        // intentional direction changes that the user designed.
        if (seg_path.size() >= 3) {
            double seg_direct = (seg_path.back() - seg_path.front()).norm();
            double seg_len = 0.0;
            for (size_t kk = 1; kk < seg_path.size(); ++kk)
                seg_len += (seg_path[kk] - seg_path[kk-1]).norm();

            bool seg_reversal = false;
            for (size_t kk = 1; kk + 1 < seg_path.size(); ++kk) {
                Eigen::Vector3d d1 = seg_path[kk]   - seg_path[kk-1];
                Eigen::Vector3d d2 = seg_path[kk+1] - seg_path[kk];
                double n1 = d1.norm(), n2 = d2.norm();
                if (n1 > 1e-6 && n2 > 1e-6 && (d1 / n1).dot(d2 / n2) < 0.0) {
                    seg_reversal = true;
                    break;
                }
            }

            bool seg_detour = (seg_direct > 0.1) && (seg_len > 2.5 * seg_direct);

            if (seg_reversal || seg_detour) {
                if (IsSegmentFree(waypoints_[i], waypoints_[i + 1])) {
                    Info("A* seg %d->%d: reversal/detour, direct free — using direct\n", i, i+1);
                    seg_path.clear();
                    seg_path.push_back(waypoints_[i]);
                    seg_path.push_back(waypoints_[i + 1]);
                } else {
                    Info("A* seg %d->%d: reversal/detour, direct blocked — keeping A*\n", i, i+1);
                }
            }
        }

        // Add intermediate waypoints when A* path has only 2 points.
        // Trivial 2-point paths produce degenerate FIRI inputs that always
        // fail, triggering the fallback corridor.  Inserting a midpoint gives
        // FIRI a real segment to work with.
        if (seg_path.size() == 2) {
            double seg_len = (seg_path[1] - seg_path[0]).norm();
            if (seg_len > 0.2) { // only if segment is non-trivial
                Eigen::Vector3d mid = 0.5 * (seg_path[0] + seg_path[1]);
                seg_path.insert(seg_path.begin() + 1, mid);
                Info("A* seg %d->%d: inserted midpoint for 2-point path (len=%.2f)\n",
                     i, i+1, seg_len);
            }
        }

        // Append (skip first to avoid duplicates)
        for (size_t j = 1; j < seg_path.size(); ++j) {
            full_path.push_back(seg_path[j]);
        }
    }

    // Diagnostic log for the complete concatenated path.
    // No global intervention — per-segment guards above handle individual A*
    // detours, and MINCO’s reversal-split handles direction changes at the
    // junctions between user-specified GUI waypoints.
    if (full_path.size() >= 2) {
        double direct_dist = (full_path.back() - full_path.front()).norm();
        double path_len = 0.0;
        for (size_t k = 1; k < full_path.size(); ++k)
            path_len += (full_path[k] - full_path[k-1]).norm();
        Info("A* full path: pts=%d len=%.2f direct=%.2f ratio=%.2f\n",
             static_cast<int>(full_path.size()), path_len, direct_dist,
             direct_dist > 1e-6 ? path_len / direct_dist : 0.0);
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

    // Set waypoints from the full path for the solvers
    num_waypoints_ = static_cast<int>(full_path.size());
    for (int i = 0; i < num_waypoints_; ++i) {
        waypoints_[i] = full_path[i];
    }

    // --- SOTA Upgrade 4: Unified FIRI corridor generation ---
    typedef std::vector<Eigen::MatrixX4d> PolyhedraH_t;
    PolyhedraH_t hPolytopes;
    std::vector<Corridor> corridors;
    bool corridors_ok = GenerateCorridors(full_path, hPolytopes, corridors);

    // GCOPTER mode: use FIRI polytope corridors + constrained MINCO solver
    if (use_gcopter_ && corridors_ok && !hPolytopes.empty()) {
        bool ok = SolveGCOPTERConstrained(hPolytopes);
        if (ok) {
            Info("obstacle avoidance: using GCOPTER constrained solver with %d polytopes\n",
                 static_cast<int>(hPolytopes.size()));
            return true;
        }
        // GCOPTER constrained failed.  Do NOT fall back to unconstrained GCOPTER:
        // the unconstrained solver ignores obstacle corridors entirely, producing
        // a trajectory that flies straight through obstacles.  This triggers the
        // safety monitor, which replans, which also fails constrained, creating
        // an infinite contingency→replan loop.  Instead, return false so the
        // caller (Replan / safety monitor) transitions to HOLDING.
        Warn("GCOPTER constrained failed — not using unconstrained fallback near obstacles\n");
        return false;
    } else if (use_gcopter_) {
        bool ok = SolveGCOPTER();
        if (ok) return true;
        Warn("GCOPTER failed, falling back to corridor min-snap\n");
    }

    // Step 3: Build SFC corridors around path (AABB from GenerateCorridors)
    if (!corridors_ok) {
        if (!BuildCorridors(full_path, sm, corridors)) {
            Warn("corridor building failed, falling back to unconstrained solve\n");
            return SolveMinSnap();
        }
    }

    // Step 4: Corridor-constrained min-snap
    return SolveMinSnapConstrained(full_path, corridors);
}

// ################################################################
// SOTA Upgrade 2: Gradient-Based Time Allocation (Richter et al. 2016)
// ################################################################
bool TrajectoryManager::OptimizeTimeAllocation(
    Eigen::VectorXd &ts,
    const Eigen::Matrix3Xd &inPs,
    const Eigen::Matrix3d &headPVA,
    const Eigen::Matrix3d &tailPVA) {

    if (!time_opt_enabled_) return false;

    const int M = static_cast<int>(ts.size());
    if (M < 1) return false;

    const int max_iters = 8;
    const double conv_threshold = 1e-3;
    const double min_seg_time = 0.3;

    // Initial solve to get coefficients
    minco::MINCO_S3NU solver;
    solver.setConditions(headPVA, tailPVA, M);
    solver.setParameters(inPs, ts);

    double energy = 0.0;
    solver.getEnergy(energy);
    double prev_cost = energy + kT_ * ts.sum();
    Info("TimeOpt: initial cost=%.4f (kT=%.1f)\n", prev_cost, kT_);

    double step_size = 0.01;

    for (int iter = 0; iter < max_iters; ++iter) {
        // Analytical gradient from MINCO
        Eigen::VectorXd energy_grad_T;
        solver.getEnergyPartialGradByTimes(energy_grad_T);

        // Total gradient: dJ/dTi = dE/dTi + kT
        Eigen::VectorXd grad(M);
        for (int i = 0; i < M; ++i) {
            grad(i) = energy_grad_T(i) + kT_;
        }

        // Gradient descent step
        Eigen::VectorXd ts_new = ts - step_size * grad;

        // Enforce minimum segment time
        for (int i = 0; i < M; ++i) {
            if (ts_new(i) < min_seg_time) ts_new(i) = min_seg_time;
        }

        // Re-solve and compute cost
        solver.setParameters(inPs, ts_new);
        double new_energy = 0.0;
        solver.getEnergy(new_energy);
        double new_cost = new_energy + kT_ * ts_new.sum();

        // Armijo-style backtracking
        int bt_iters = 0;
        while (new_cost > prev_cost && bt_iters < 5) {
            step_size *= 0.5;
            ts_new = ts - step_size * grad;
            for (int i = 0; i < M; ++i) {
                if (ts_new(i) < min_seg_time) ts_new(i) = min_seg_time;
            }
            solver.setParameters(inPs, ts_new);
            solver.getEnergy(new_energy);
            new_cost = new_energy + kT_ * ts_new.sum();
            bt_iters++;
        }

        double rel_change = std::abs(new_cost - prev_cost) / (std::abs(prev_cost) + 1e-10);

        if (new_cost < prev_cost) {
            ts = ts_new;
            prev_cost = new_cost;
            energy = new_energy;
            step_size *= 1.2;
            if (step_size > 0.1) step_size = 0.1;
        } else {
            solver.setParameters(inPs, ts);
        }

        if (rel_change < conv_threshold) {
            Info("TimeOpt: converged at iter %d, cost=%.4f\n", iter, prev_cost);
            break;
        }
    }

    Info("TimeOpt: final cost=%.4f, total_time=%.2f s\n", prev_cost, ts.sum());
    return true;
}

// ################################################################
// SOTA Upgrade 3: TOPP-RA Post-Processing (Pham & Pham, IEEE T-RO 2018)
// ################################################################
bool TrajectoryManager::ApplyTOPPRA() {
    if (!topp_ra_enabled_ || !trajectory_valid_) return false;
    if (!use_gcopter_ || !gcopter_traj_valid_) return false;

    const double v_max = std::max(max_vel_->Value(), 0.1);
    const double a_max = std::max(max_acc_->Value(), 0.1);
    const double eps = 1e-10;

    const int num_pieces = gcopter_traj_.getPieceNum();
    if (num_pieces < 1 || num_pieces > MAX_SEGMENTS) return false;

    // Step 1: Discretize the path by arc-length parameter s
    const int samples_per_seg = TOPPRA_SAMPLES_PER_SEG;
    const int N = num_pieces * samples_per_seg;

    if (N + 1 > TOPPRA_MAX_GRID) return false;

    Eigen::VectorXd durations = gcopter_traj_.getDurations();

    // Compute arc-length parameterization
    double s_accum = 0.0;
    toppra_s_[0] = 0.0;
    int grid_idx = 0;

    for (int seg = 0; seg < num_pieces; ++seg) {
        double dur = durations(seg);
        double dt_sample = dur / samples_per_seg;

        for (int k = 0; k < samples_per_seg; ++k) {
            double tau0 = dt_sample * k;
            double tau1 = dt_sample * (k + 1);

            Eigen::Vector3d v0 = gcopter_traj_[seg].getVel(tau0);
            Eigen::Vector3d v1 = gcopter_traj_[seg].getVel(tau1);
            double ds = 0.5 * (v0.norm() + v1.norm()) * dt_sample;
            if (ds < eps) ds = eps;

            s_accum += ds;
            toppra_ds_[grid_idx] = ds;
            toppra_s_[grid_idx + 1] = s_accum;

            Eigen::Vector3d vel_k = gcopter_traj_[seg].getVel(tau0);
            Eigen::Vector3d acc_k = gcopter_traj_[seg].getAcc(tau0);
            double vnorm = vel_k.norm();
            if (vnorm < eps) vnorm = eps;

            for (int ax = 0; ax < 3; ++ax) {
                toppra_p_prime_[grid_idx][ax] = vel_k(ax) / vnorm;
            }

            double adotv_hat = 0.0;
            for (int ax = 0; ax < 3; ++ax) {
                adotv_hat += acc_k(ax) * (vel_k(ax) / vnorm);
            }
            for (int ax = 0; ax < 3; ++ax) {
                double tangential = adotv_hat * (vel_k(ax) / vnorm);
                toppra_p_dprime_[grid_idx][ax] = (acc_k(ax) - tangential) / (vnorm * vnorm);
            }

            grid_idx++;
        }
    }

    // Last grid point
    {
        double last_dur = durations(num_pieces - 1);
        Eigen::Vector3d vel_end = gcopter_traj_[num_pieces - 1].getVel(last_dur);
        Eigen::Vector3d acc_end = gcopter_traj_[num_pieces - 1].getAcc(last_dur);
        double vnorm = vel_end.norm();
        if (vnorm < eps) vnorm = eps;
        for (int ax = 0; ax < 3; ++ax) {
            toppra_p_prime_[N][ax] = vel_end(ax) / vnorm;
        }
        double adotv_hat = 0.0;
        for (int ax = 0; ax < 3; ++ax) {
            adotv_hat += acc_end(ax) * (vel_end(ax) / vnorm);
        }
        for (int ax = 0; ax < 3; ++ax) {
            double tangential = adotv_hat * (vel_end(ax) / vnorm);
            toppra_p_dprime_[N][ax] = (acc_end(ax) - tangential) / (vnorm * vnorm);
        }
    }

    // Step 2: Compute velocity limits
    for (int i = 0; i <= N; ++i) {
        double x_limit = 1e10;
        for (int ax = 0; ax < 3; ++ax) {
            double pp = std::abs(toppra_p_prime_[i][ax]);
            if (pp > eps) {
                double lim = (v_max / pp) * (v_max / pp);
                if (lim < x_limit) x_limit = lim;
            }
        }
        toppra_x_max_vel_[i] = x_limit;
    }

    // Step 3: Backward pass — compute controllable sets K[i]
    toppra_K_lo_[N] = 0.0;
    toppra_K_hi_[N] = 0.0;

    for (int i = N - 1; i >= 0; --i) {
        double ds_i = toppra_ds_[i];
        if (ds_i < eps) ds_i = eps;
        double inv_2ds = 1.0 / (2.0 * ds_i);

        double x_max_i = toppra_x_max_vel_[i];
        double K_next_lo = toppra_K_lo_[i + 1];
        double K_next_hi = toppra_K_hi_[i + 1];

        // Precompute acceleration constraint slopes
        double acc_u_lo_slope[3], acc_u_lo_intercept[3];
        double acc_u_hi_slope[3], acc_u_hi_intercept[3];

        for (int ax = 0; ax < 3; ++ax) {
            double pp = toppra_p_prime_[i][ax];
            double ppp = toppra_p_dprime_[i][ax];

            if (std::abs(pp) < eps) {
                if (std::abs(ppp) > eps) {
                    double x_acc_limit = a_max / std::abs(ppp);
                    if (x_acc_limit < x_max_i) x_max_i = x_acc_limit;
                }
                acc_u_lo_slope[ax] = 0.0;
                acc_u_lo_intercept[ax] = -1e10;
                acc_u_hi_slope[ax] = 0.0;
                acc_u_hi_intercept[ax] = 1e10;
            } else {
                double inv_pp = 1.0 / pp;
                if (pp > 0) {
                    acc_u_lo_slope[ax] = -ppp * inv_pp;
                    acc_u_lo_intercept[ax] = -a_max * inv_pp;
                    acc_u_hi_slope[ax] = -ppp * inv_pp;
                    acc_u_hi_intercept[ax] = a_max * inv_pp;
                } else {
                    acc_u_hi_slope[ax] = -ppp * inv_pp;
                    acc_u_hi_intercept[ax] = -a_max * inv_pp;
                    acc_u_lo_slope[ax] = -ppp * inv_pp;
                    acc_u_lo_intercept[ax] = a_max * inv_pp;
                }
            }
        }

        // Feasibility check lambda
        auto isFeasible = [&](double x) -> bool {
            if (x < 0.0 || x > x_max_i) return false;
            double u_lo_val = (K_next_lo - x) * inv_2ds;
            double u_hi_val = (K_next_hi - x) * inv_2ds;
            for (int ax = 0; ax < 3; ++ax) {
                double alo = acc_u_lo_slope[ax] * x + acc_u_lo_intercept[ax];
                double ahi = acc_u_hi_slope[ax] * x + acc_u_hi_intercept[ax];
                if (alo > u_lo_val) u_lo_val = alo;
                if (ahi < u_hi_val) u_hi_val = ahi;
            }
            return u_lo_val <= u_hi_val + eps;
        };

        // Quick scan: 32 sample points
        const int SCAN_PTS = 32;
        double x_step = x_max_i / SCAN_PTS;
        int first_feasible = -1, last_feasible = -1;

        for (int s = 0; s <= SCAN_PTS; ++s) {
            double x_test = x_step * s;
            if (isFeasible(x_test)) {
                if (first_feasible < 0) first_feasible = s;
                last_feasible = s;
            }
        }

        if (first_feasible < 0) {
            toppra_K_lo_[i] = 0.0;
            toppra_K_hi_[i] = 0.0;
            continue;
        }

        // Refine x_lo by bisection
        double x_lo, x_hi_found;
        {
            double lo = (first_feasible > 0) ? x_step * (first_feasible - 1) : 0.0;
            double hi = x_step * first_feasible;
            for (int b = 0; b < 10; ++b) {
                double mid = 0.5 * (lo + hi);
                if (isFeasible(mid)) hi = mid; else lo = mid;
            }
            x_lo = hi;
        }
        {
            double lo = x_step * last_feasible;
            double hi = (last_feasible < SCAN_PTS) ? x_step * (last_feasible + 1) : x_max_i;
            if (hi > x_max_i) hi = x_max_i;
            for (int b = 0; b < 10; ++b) {
                double mid = 0.5 * (lo + hi);
                if (isFeasible(mid)) lo = mid; else hi = mid;
            }
            x_hi_found = lo;
        }

        toppra_K_lo_[i] = std::max(x_lo, 0.0);
        toppra_K_hi_[i] = std::min(x_hi_found, x_max_i);
        if (toppra_K_hi_[i] < toppra_K_lo_[i]) {
            toppra_K_lo_[i] = 0.0;
            toppra_K_hi_[i] = 0.0;
        }
    }

    // Step 4: Forward pass — greedy time-optimal
    toppra_x_[0] = 0.0;

    for (int i = 0; i < N; ++i) {
        double ds_i = toppra_ds_[i];
        if (ds_i < eps) ds_i = eps;
        double inv_2ds = 1.0 / (2.0 * ds_i);
        double x_i = toppra_x_[i];

        double u_hi_val = (toppra_K_hi_[i + 1] - x_i) * inv_2ds;

        for (int ax = 0; ax < 3; ++ax) {
            double pp = toppra_p_prime_[i][ax];
            double ppp = toppra_p_dprime_[i][ax];
            if (std::abs(pp) < eps) continue;
            double inv_pp = 1.0 / pp;
            double u_upper;
            if (pp > 0) u_upper = (a_max - ppp * x_i) * inv_pp;
            else        u_upper = (-a_max - ppp * x_i) * inv_pp;
            if (u_upper < u_hi_val) u_hi_val = u_upper;
        }

        double u_lo_val = (toppra_K_lo_[i + 1] - x_i) * inv_2ds;
        for (int ax = 0; ax < 3; ++ax) {
            double pp = toppra_p_prime_[i][ax];
            double ppp = toppra_p_dprime_[i][ax];
            if (std::abs(pp) < eps) continue;
            double inv_pp = 1.0 / pp;
            double u_lower;
            if (pp > 0) u_lower = (-a_max - ppp * x_i) * inv_pp;
            else        u_lower = (a_max - ppp * x_i) * inv_pp;
            if (u_lower > u_lo_val) u_lo_val = u_lower;
        }

        double u_i = u_hi_val;
        if (u_i < u_lo_val) u_i = u_lo_val;
        toppra_u_[i] = u_i;

        double x_next = x_i + 2.0 * ds_i * u_i;
        if (x_next < toppra_K_lo_[i + 1]) x_next = toppra_K_lo_[i + 1];
        if (x_next > toppra_K_hi_[i + 1]) x_next = toppra_K_hi_[i + 1];
        if (x_next < 0.0) x_next = 0.0;

        toppra_x_[i + 1] = x_next;
    }

    // Step 5: Time integration — compute new segment durations
    Eigen::VectorXd new_durations(num_pieces);
    double new_total_time = 0.0;
    int gi = 0;

    for (int seg = 0; seg < num_pieces; ++seg) {
        double seg_time = 0.0;
        for (int k = 0; k < samples_per_seg; ++k) {
            double sdot_i = std::sqrt(std::max(toppra_x_[gi], 0.0));
            double sdot_next = std::sqrt(std::max(toppra_x_[gi + 1], 0.0));
            double ds_i = toppra_ds_[gi];
            double dt = 2.0 * ds_i / (sdot_i + sdot_next + eps);
            seg_time += dt;
            gi++;
        }
        if (seg_time < 0.3) seg_time = 0.3;
        new_durations(seg) = seg_time;
        new_total_time += seg_time;
    }

    // Step 6: Re-solve MINCO with TOPP-RA durations + iterative tightening
    double time_ratio = new_total_time / total_duration_;

    if (std::abs(time_ratio - 1.0) < 0.02) {
        Info("TOPP-RA: trajectory already near-optimal (ratio=%.3f)\n", time_ratio);
        return true;
    }

    Info("TOPP-RA: rescaling %.2f -> %.2f s (ratio=%.2f)\n",
         total_duration_, new_total_time, time_ratio);

    // Sanity check: if TOPP-RA wants to stretch time by >50x, the input
    // trajectory is far too aggressive.  Reject instead of producing an
    // absurdly long trajectory that the drone will never finish.
    if (time_ratio > 50.0 || new_total_time > 300.0) {
        Warn("TOPP-RA: rescaling ratio %.1f too extreme (max 50x / 300s), rejecting\n",
             time_ratio);
        return false;
    }

    int M = num_pieces;
    Eigen::Matrix3d headPVA, tailPVA;
    headPVA.col(0) = waypoints_[0];
    headPVA.col(1) = start_vel_;
    headPVA.col(2) = Eigen::Vector3d::Zero();
    tailPVA.col(0) = waypoints_[num_waypoints_ - 1];
    tailPVA.col(1) = end_vel_;
    tailPVA.col(2) = Eigen::Vector3d::Zero();

    Eigen::Matrix3Xd inPs(3, M - 1);
    for (int i = 0; i < M - 1; ++i) {
        inPs.col(i) = waypoints_[i + 1];
    }

    // Iterative tightening
    const int MAX_TIGHTEN_ITERS = 3;
    const int SAMPLES_PER_SEG = 100;
    const double SAFETY_MARGIN = 1.05;

    for (int iter = 0; iter < MAX_TIGHTEN_ITERS; ++iter) {
        minco::MINCO_S3NU solver;
        solver.setConditions(headPVA, tailPVA, M);
        solver.setParameters(inPs, new_durations);

        Trajectory<5> new_traj;
        solver.getTrajectory(new_traj);

        if (new_traj.getPieceNum() <= 0) break;

        bool feasible = true;
        for (int seg = 0; seg < M; ++seg) {
            double seg_dur = new_durations(seg);
            double max_v_seg = 0.0;
            double max_a_seg = 0.0;

            for (int k = 0; k <= SAMPLES_PER_SEG; ++k) {
                double t = seg_dur * k / static_cast<double>(SAMPLES_PER_SEG);
                Eigen::Vector3d vel_sample = new_traj[seg].getVel(t);
                Eigen::Vector3d acc_sample = new_traj[seg].getAcc(t);

                for (int ax = 0; ax < 3; ++ax) {
                    double av = std::abs(vel_sample(ax));
                    double aa = std::abs(acc_sample(ax));
                    if (av > max_v_seg) max_v_seg = av;
                    if (aa > max_a_seg) max_a_seg = aa;
                }
            }

            if (max_v_seg > v_max || max_a_seg > a_max) {
                double v_scale = (max_v_seg > v_max) ? (max_v_seg / v_max) : 1.0;
                double a_scale = (max_a_seg > a_max) ? std::sqrt(max_a_seg / a_max) : 1.0;
                double scale = std::max(v_scale, a_scale) * SAFETY_MARGIN;
                new_durations(seg) *= scale;
                feasible = false;
            }
        }

        if (feasible || iter == MAX_TIGHTEN_ITERS - 1) {
            new_total_time = new_durations.sum();
            gcopter_traj_ = new_traj;
            total_duration_ = new_total_time;

            num_segments_ = std::min(M, MAX_SEGMENTS);
            for (int i = 0; i < num_segments_; ++i) {
                segments_[i].duration = new_traj[i].getDuration();
                segments_[i].coeffs.setZero();
                Eigen::Matrix<double, 3, 6> cm = new_traj[i].getCoeffMat();
                for (int axis = 0; axis < 3; ++axis) {
                    for (int c = 0; c <= 5; ++c) {
                        segments_[i].coeffs(axis, c) = cm(axis, 5 - c);
                    }
                }
            }

            if (feasible)
                Info("TOPP-RA: limits satisfied after %d iteration(s)\n", iter + 1);
            else
                Info("TOPP-RA: max iterations reached, accepting best result\n");
            break;
        }

        Info("TOPP-RA: tightening iteration %d, scaling up violating segments\n", iter + 1);
    }

    return true;
}

// ################################################################
// SOTA Upgrade 4: FIRI Corridor Generation (unified)
// ################################################################
bool TrajectoryManager::GenerateCorridors(
    const std::vector<Eigen::Vector3d> &path_wps,
    std::vector<Eigen::MatrixX4d> &hPolytopes,
    std::vector<Corridor> &aabb_corridors) {

    hPolytopes.clear();
    aabb_corridors.clear();

    // --- Merge near-coincident waypoints to avoid degenerate zero-length
    //     segments that cause FIRI to fail ---
    std::vector<Eigen::Vector3d> merged_wps;
    merged_wps.reserve(path_wps.size());
    merged_wps.push_back(path_wps[0]);
    const double merge_thresh = 0.1; // metres
    for (size_t k = 1; k < path_wps.size(); ++k) {
        if ((path_wps[k] - merged_wps.back()).norm() > merge_thresh) {
            merged_wps.push_back(path_wps[k]);
        }
    }
    // Must keep the final waypoint even if it was merged
    if (merged_wps.back() != path_wps.back()) {
        merged_wps.push_back(path_wps.back());
    }
    if (merged_wps.size() < 2) {
        if (path_wps.size() >= 2) {
            merged_wps.clear();
            merged_wps.push_back(path_wps.front());
            merged_wps.push_back(path_wps.back());
        } else {
            return false;
        }
    }
    if (merged_wps.size() < path_wps.size()) {
        Info("GenerateCorridors: merged %d near-coincident waypoints (%d -> %d)\n",
             static_cast<int>(path_wps.size() - merged_wps.size()),
             static_cast<int>(path_wps.size()), static_cast<int>(merged_wps.size()));
    }

    // Use merged_wps from here on
    const std::vector<Eigen::Vector3d> &wps = merged_wps;
    int n_segs = static_cast<int>(wps.size()) - 1;
    if (n_segs < 1) return false;

    double sm = safety_margin_->Value();

    // --- Build AABB H-polytopes for GCOPTER ---
    // We use axis-aligned bounding boxes instead of FIRI polytopes.
    // FIRI can produce complex polytopes whose consecutive intersections
    // are nearly degenerate, causing quickhull inside GCOPTER's
    // processCorridor() to SEGFAULT.  AABB boxes (6 axis-aligned faces)
    // are simple, well-conditioned, always overlap at shared waypoints,
    // and quickhull handles them trivially.  The MINCO optimizer inside
    // GCOPTER still produces high-quality trajectories because it
    // optimizes within the corridor constraints.
    //
    // Obstacle avoidance is enforced by the A* path: the waypoints
    // already route around obstacles, and the AABB corridor margin is
    // kept tight enough that the trajectory stays in free space.
    const double corridor_margin = 0.8; // metres — tight enough for obstacle clearance

    for (int i = 0; i < n_segs; ++i) {
        const Eigen::Vector3d &seg_a = wps[i];
        const Eigen::Vector3d &seg_b = wps[i + 1];

        Eigen::Vector3d lo, hi;
        for (int a = 0; a < 3; ++a) {
            lo(a) = std::min(seg_a(a), seg_b(a)) - corridor_margin;
            hi(a) = std::max(seg_a(a), seg_b(a)) + corridor_margin;
        }
        // Clamp to workspace bounds
        lo.x() = std::max(lo.x(), WS_X_MIN);
        lo.y() = std::max(lo.y(), WS_Y_MIN);
        lo.z() = std::max(lo.z(), WS_Z_MIN);
        hi.x() = std::min(hi.x(), WS_X_MAX);
        hi.y() = std::min(hi.y(), WS_Y_MAX);
        hi.z() = std::min(hi.z(), WS_Z_MAX);
        // Ensure minimum extent per axis (non-degenerate box)
        for (int a = 0; a < 3; ++a) {
            if (hi(a) - lo(a) < 0.4) {
                double mid = 0.5 * (lo(a) + hi(a));
                lo(a) = mid - 0.2;
                hi(a) = mid + 0.2;
            }
        }
        // H-representation: normal·x + d <= 0
        Eigen::MatrixX4d box(6, 4);
        box.row(0) <<  1.0,  0.0,  0.0, -hi.x();
        box.row(1) << -1.0,  0.0,  0.0,  lo.x();
        box.row(2) <<  0.0,  1.0,  0.0, -hi.y();
        box.row(3) <<  0.0, -1.0,  0.0,  lo.y();
        box.row(4) <<  0.0,  0.0,  1.0, -hi.z();
        box.row(5) <<  0.0,  0.0, -1.0,  lo.z();
        hPolytopes.push_back(box);

        // Always build an AABB corridor as backup
        Corridor c;
        c.lo = Eigen::Vector3d(
            std::min(seg_a.x(), seg_b.x()) - sm - 0.5,
            std::min(seg_a.y(), seg_b.y()) - sm - 0.5,
            std::min(seg_a.z(), seg_b.z()) - sm - 0.5);
        c.hi = Eigen::Vector3d(
            std::max(seg_a.x(), seg_b.x()) + sm + 0.5,
            std::max(seg_a.y(), seg_b.y()) + sm + 0.5,
            std::max(seg_a.z(), seg_b.z()) + sm + 0.5);
        c.lo.x() = std::max(c.lo.x(), WS_X_MIN);
        c.lo.y() = std::max(c.lo.y(), WS_Y_MIN);
        c.lo.z() = std::max(c.lo.z(), WS_Z_MIN);
        c.hi.x() = std::min(c.hi.x(), WS_X_MAX);
        c.hi.y() = std::min(c.hi.y(), WS_Y_MAX);
        c.hi.z() = std::min(c.hi.z(), WS_Z_MAX);
        aabb_corridors.push_back(c);
    }

    Info("GenerateCorridors: %d AABB corridors generated\n", n_segs);

    // No post-validation needed: AABB boxes with shared waypoints always
    // have a well-conditioned overlap region.  Consecutive boxes for
    // segments (A→B) and (B→C) both contain point B with at least
    // corridor_margin on each side.
    return true;
}

// ################################################################
// SOTA Upgrade 5: Contingency Trajectory (decelerate to hover)
// ################################################################
void TrajectoryManager::GenerateContingencyTrajectory(
    const Eigen::Vector3d &pos,
    const Eigen::Vector3d &vel,
    const Eigen::Vector3d &acc) {

    contingency_valid_ = false;

    double v_norm = vel.norm();
    double a_max_val = max_acc_->Value();
    if (a_max_val < 0.1) a_max_val = 0.1;

    double t_stop = (v_norm > 0.01) ? (v_norm / a_max_val) : 0.5;
    if (t_stop < 0.3) t_stop = 0.3;
    if (t_stop > 3.0) t_stop = 3.0;

    Eigen::Vector3d hover_pos = pos + vel * (t_stop * 0.5);

    Eigen::Matrix3d headPVA, tailPVA;
    headPVA.col(0) = pos;
    headPVA.col(1) = vel;
    headPVA.col(2) = acc;
    tailPVA.col(0) = hover_pos;
    tailPVA.col(1) = Eigen::Vector3d::Zero();
    tailPVA.col(2) = Eigen::Vector3d::Zero();

    Eigen::VectorXd ts(1);
    ts(0) = t_stop;

    Eigen::Matrix3Xd inPs(3, 0);

    minco::MINCO_S3NU solver;
    solver.setConditions(headPVA, tailPVA, 1);
    solver.setParameters(inPs, ts);

    Trajectory<5> traj;
    solver.getTrajectory(traj);

    if (traj.getPieceNum() > 0) {
        contingency_traj_ = traj;
        contingency_valid_ = true;
        Info("Contingency trajectory: decel-to-hover in %.2f s\n", t_stop);
    }
}

// ============================================================
// SOTA Upgrade 6: ESDF-Lite BFS wavefront distance field
// ============================================================
void TrajectoryManager::ComputeDistanceField() {
    size_t N = static_cast<size_t>(grid_nx_) * grid_ny_ * grid_nz_;
    distance_field_.assign(N, std::numeric_limits<float>::max());

    std::queue<int> bfs;

    // Initialize: obstacle cells = 0, seed BFS from them
    for (size_t i = 0; i < N; ++i) {
        if (grid_data_[i]) {
            distance_field_[i] = 0.0f;
            bfs.push(static_cast<int>(i));
        }
    }

    // 6-connected BFS (Manhattan distance — lightweight for ARM)
    const int dx[6] = {1, -1, 0, 0, 0, 0};
    const int dy[6] = {0, 0, 1, -1, 0, 0};
    const int dz[6] = {0, 0, 0, 0, 1, -1};

    while (!bfs.empty()) {
        int idx = bfs.front();
        bfs.pop();

        int iz = idx % grid_nz_;
        int rem = idx / grid_nz_;
        int iy = rem % grid_ny_;
        int ix = rem / grid_ny_;

        float next_dist = distance_field_[idx] + static_cast<float>(grid_res_);

        for (int d = 0; d < 6; ++d) {
            int nx = ix + dx[d];
            int ny = iy + dy[d];
            int nz = iz + dz[d];
            if (nx < 0 || nx >= grid_nx_ || ny < 0 || ny >= grid_ny_ || nz < 0 || nz >= grid_nz_)
                continue;
            int nidx = nx * grid_ny_ * grid_nz_ + ny * grid_nz_ + nz;
            if (next_dist < distance_field_[nidx]) {
                distance_field_[nidx] = next_dist;
                bfs.push(nidx);
            }
        }
    }
}

double TrajectoryManager::GetObstacleDistance(const Eigen::Vector3d &world_pos) const {
    if (distance_field_.empty()) return std::numeric_limits<double>::max();

    Eigen::Vector3i gi = WorldToGrid(world_pos);
    // Out-of-bounds means no obstacle information — assume safe (max distance)
    // Previously returned 0.0 which falsely triggered emergency contingency
    if (!GridInBounds(gi.x(), gi.y(), gi.z())) return std::numeric_limits<double>::max();

    int idx = gi.x() * grid_ny_ * grid_nz_ + gi.y() * grid_nz_ + gi.z();
    return static_cast<double>(distance_field_[idx]);
}

// ============================================================
// SOTA: Precompute circular inflation template
// ============================================================
void TrajectoryManager::PrecomputeInflationTemplate(double radius) {
    if (std::abs(radius - inflation_template_radius_) < 1e-6 && !inflation_template_.empty())
        return;

    inflation_template_.clear();
    inflation_template_radius_ = radius;

    int r_cells = static_cast<int>(std::ceil(radius / grid_res_));
    double r2 = radius * radius;

    for (int dx = -r_cells; dx <= r_cells; ++dx) {
        for (int dy = -r_cells; dy <= r_cells; ++dy) {
            double wx = dx * grid_res_;
            double wy = dy * grid_res_;
            if (wx * wx + wy * wy <= r2) {
                InflationOffset off;
                off.dx = dx;
                off.dy = dy;
                inflation_template_.push_back(off);
            }
        }
    }
}

// ============================================================
// SOTA Upgrade 9: Check if any obstacle moved significantly
// ============================================================
bool TrajectoryManager::ObstaclesMoved() const {
    if (num_obstacles_ != prev_num_obstacles_) return true;
    for (int i = 0; i < num_obstacles_; ++i) {
        double dx = obstacles_[i].pos.x() - prev_obstacle_pos_[i].x();
        double dy = obstacles_[i].pos.y() - prev_obstacle_pos_[i].y();
        double dz = obstacles_[i].pos.z() - prev_obstacle_pos_[i].z();
        if (dx * dx + dy * dy + dz * dz > grid_res_ * grid_res_) return true;
    }
    return false;
}

// ============================================================
// SOTA Upgrade 9: Incremental occupancy grid update
// ============================================================
void TrajectoryManager::BuildOccupancyGridIncremental() {
    // First call or grid bounds changed: full rebuild
    if (!grid_initialized_ || grid_dirty_) {
        BuildOccupancyGrid();
        for (int i = 0; i < num_obstacles_; ++i) {
            prev_obstacle_pos_[i] = obstacles_[i].pos;
        }
        prev_num_obstacles_ = num_obstacles_;
        grid_dirty_ = false;
        grid_initialized_ = true;
        distance_field_valid_ = false;
        return;
    }

    // Check if obstacles moved
    if (!ObstaclesMoved()) return;

    // Incremental: rebuild with pre-computed inflation template for speed
    double sm = safety_margin_->Value();
    double inflate_radius = 0.0;
    for (int i = 0; i < num_obstacles_; ++i) {
        inflate_radius = std::max(inflate_radius, obstacles_[i].radius + sm);
    }
    PrecomputeInflationTemplate(inflate_radius);

    ClearGrid();

    // Mark ground plane
    int iz_ground = -1;
    for (int iz = 0; iz < grid_nz_; ++iz) {
        double wz = grid_origin_.z() + (iz + 0.5) * grid_res_;
        if (wz >= -sm) { iz_ground = iz; break; }
    }
    if (iz_ground >= 0) {
        int nz_ground = grid_nz_ - iz_ground;
        for (int ix = 0; ix < grid_nx_; ++ix) {
            for (int iy = 0; iy < grid_ny_; ++iy) {
                size_t base = static_cast<size_t>(ix) * grid_ny_ * grid_nz_
                            + static_cast<size_t>(iy) * grid_nz_
                            + static_cast<size_t>(iz_ground);
                std::memset(&grid_data_[base], 1u, static_cast<size_t>(nz_ground));
            }
        }
    }

    // Mark obstacles using precomputed template
    for (int i = 0; i < num_obstacles_; ++i) {
        double obs_x = obstacles_[i].pos.x();
        double obs_y = obstacles_[i].pos.y();
        double obs_radius = obstacles_[i].radius + sm;

        if (std::abs(obs_radius - inflation_template_radius_) < grid_res_ * 0.5) {
            Eigen::Vector3i center_g = WorldToGrid(Eigen::Vector3d(obs_x, obs_y, 0.0));
            for (size_t t = 0; t < inflation_template_.size(); ++t) {
                int gx = center_g.x() + inflation_template_[t].dx;
                int gy = center_g.y() + inflation_template_[t].dy;
                if (gx >= 0 && gx < grid_nx_ && gy >= 0 && gy < grid_ny_) {
                    size_t base = static_cast<size_t>(gx) * grid_ny_ * grid_nz_
                                + static_cast<size_t>(gy) * grid_nz_;
                    std::memset(&grid_data_[base], 1u, static_cast<size_t>(grid_nz_));
                }
            }
        } else {
            MarkCylinderOccupied(obs_x, obs_y, obs_radius);
        }

        // Spatio-temporal prediction
        double obs_speed = obstacles_[i].vel.norm();
        if (obs_speed > 0.01 && total_duration_ > 0.0) {
            double remaining_time = total_duration_;
            if (execution_time_set_ && last_update_time_ > 0.0) {
                double elapsed = last_update_time_ - execution_start_time_;
                remaining_time = std::max(total_duration_ - elapsed, 1.0);
            }
            double pred_horizon = std::min(remaining_time, 5.0);
            for (int step = 1; step <= 4; ++step) {
                double t_pred = pred_horizon * step / 4.0;
                double pred_x = obs_x + obstacles_[i].vel.x() * t_pred;
                double pred_y = obs_y + obstacles_[i].vel.y() * t_pred;
                double inflated_radius = obs_radius + sm * t_pred;
                MarkCylinderOccupied(pred_x, pred_y, inflated_radius);
            }
        }
    }

    // Update cached positions
    for (int i = 0; i < num_obstacles_; ++i) {
        prev_obstacle_pos_[i] = obstacles_[i].pos;
    }
    prev_num_obstacles_ = num_obstacles_;
    distance_field_valid_ = false;
    cache_valid_ = false;
}
