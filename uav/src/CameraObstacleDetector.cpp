// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2024
//  filename:   CameraObstacleDetector.cpp
//
//  author:     Sergio Urzua
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  purpose:    Simple color-based obstacle detection from UAV camera
//
/*********************************************************************/

#include "CameraObstacleDetector.h"
#include <GroupBox.h>
#include <ComboBox.h>
#include <DoubleSpinBox.h>
#include <LayoutPosition.h>
#include <cmath>
#include <algorithm>

using namespace flair::gui;
using namespace flair::core;

CameraObstacleDetector::CameraObstacleDetector(
    const LayoutPosition *position,
    const std::string &name)
    : obstacle_detected_(false),
      obstacle_pos_(0, 0, 0),
      obstacle_radius_(0.0f)
{
    settings_box_ = new GroupBox(position, name.c_str());

    enabled_mode_ = new ComboBox(settings_box_->NewRow(), "Camera obstacles");
    enabled_mode_->AddItem("Disabled");
    enabled_mode_->AddItem("Enabled");

    marker_color_ = new ComboBox(settings_box_->LastRowLastCol(), "Marker color");
    marker_color_->AddItem("Red");
    marker_color_->AddItem("Green");
    marker_color_->AddItem("Blue");

    marker_size_cm_ = new DoubleSpinBox(settings_box_->NewRow(), "Marker size", " cm",
                                         1.0, 100.0, 1.0, 1);
}

CameraObstacleDetector::~CameraObstacleDetector() {
}

bool CameraObstacleDetector::IsEnabled() const {
    return enabled_mode_->CurrentIndex() == 1;
}

CameraObstacleDetector::HSVRange CameraObstacleDetector::GetColorRange() const {
    HSVRange range;
    switch (marker_color_->CurrentIndex()) {
    case 0: // Red
        range.h_min = 0;   range.h_max = 10;
        range.s_min = 100; range.s_max = 255;
        range.v_min = 100; range.v_max = 255;
        break;
    case 1: // Green
        range.h_min = 35;  range.h_max = 85;
        range.s_min = 100; range.s_max = 255;
        range.v_min = 50;  range.v_max = 255;
        break;
    case 2: // Blue
        range.h_min = 100; range.h_max = 130;
        range.s_min = 100; range.s_max = 255;
        range.v_min = 50;  range.v_max = 255;
        break;
    default:
        range.h_min = 0; range.h_max = 180;
        range.s_min = 0; range.s_max = 255;
        range.v_min = 0; range.v_max = 255;
        break;
    }
    return range;
}

void CameraObstacleDetector::RGBtoHSV(uint8_t r, uint8_t g, uint8_t b,
                                        uint8_t &h, uint8_t &s, uint8_t &v) {
    float rf = r / 255.0f;
    float gf = g / 255.0f;
    float bf = b / 255.0f;
    float maxc = std::max(rf, std::max(gf, bf));
    float minc = std::min(rf, std::min(gf, bf));
    float diff = maxc - minc;

    v = static_cast<uint8_t>(maxc * 255.0f);

    if (maxc < 1e-6f) {
        s = 0;
        h = 0;
        return;
    }
    s = static_cast<uint8_t>((diff / maxc) * 255.0f);

    float hf = 0.0f;
    if (diff < 1e-6f) {
        hf = 0.0f;
    } else if (maxc == rf) {
        hf = 60.0f * std::fmod((gf - bf) / diff, 6.0f);
    } else if (maxc == gf) {
        hf = 60.0f * ((bf - rf) / diff + 2.0f);
    } else {
        hf = 60.0f * ((rf - gf) / diff + 4.0f);
    }
    if (hf < 0.0f) hf += 360.0f;
    // OpenCV-style: H is [0..180]
    h = static_cast<uint8_t>(hf / 2.0f);
}

void CameraObstacleDetector::Update() {
    if (!IsEnabled()) {
        obstacle_detected_ = false;
        return;
    }

    // STUB: In a real implementation, this would:
    // 1. Capture a frame from the Flair V4L2Camera interface
    // 2. Convert each pixel from RGB to HSV
    // 3. Threshold using GetColorRange()
    // 4. Find the largest contiguous blob (connected component)
    // 5. Compute bounding box center and area
    // 6. Estimate distance from known marker size:
    //    distance = (marker_size_m * focal_length) / blob_width_pixels

    // For now, report no detection.
    // When camera interface is available, replace this stub:
    //
    //   HSVRange range = GetColorRange();
    //   double marker_size_m = marker_size_cm_->Value() / 100.0;
    //   // focal_length ~= 550 pixels for AR.Drone 2.0 720p camera
    //   double focal_length = 550.0;
    //   // Scan frame pixels, threshold, find largest blob...
    //   // if (blob_found) {
    //   //     double distance = marker_size_m * focal_length / blob_width;
    //   //     double bearing_x = (blob_cx - img_width/2.0) / focal_length;
    //   //     double bearing_y = (blob_cy - img_height/2.0) / focal_length;
    //   //     obstacle_pos_ = Vector3Df(distance, -bearing_x * distance, -bearing_y * distance);
    //   //     obstacle_radius_ = marker_size_m / 2.0f;
    //   //     obstacle_detected_ = true;
    //   // }

    obstacle_detected_ = false;
}

bool CameraObstacleDetector::IsObstacleDetected() const {
    return obstacle_detected_;
}

void CameraObstacleDetector::GetObstaclePosition(Vector3Df &pos) const {
    pos = obstacle_pos_;
}

float CameraObstacleDetector::GetObstacleRadius() const {
    return obstacle_radius_;
}
