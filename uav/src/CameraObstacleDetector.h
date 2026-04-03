// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file CameraObstacleDetector.h
 * \brief Simple color-based obstacle detection from UAV camera
 * \author Sergio Urzua, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2024
 */

#ifndef CAMERA_OBSTACLE_DETECTOR_H
#define CAMERA_OBSTACLE_DETECTOR_H

#include <Vector3D.h>
#include <string>
#include <cstdint>

namespace flair {
    namespace gui {
        class LayoutPosition;
        class ComboBox;
        class DoubleSpinBox;
        class GroupBox;
    }
}

/*!
 * \class CameraObstacleDetector
 * \brief Lightweight color-based blob detection for obstacle avoidance
 *
 * Captures frames from the UAV's frontal camera, applies HSV color
 * thresholding, finds the largest blob, and estimates distance from
 * known physical marker size.
 *
 * NOTE: Camera capture is stubbed — the actual V4L2/Flair camera
 * interface should be connected when available on target hardware.
 */
class CameraObstacleDetector {
public:
    CameraObstacleDetector(const flair::gui::LayoutPosition *position,
                           const std::string &name);
    ~CameraObstacleDetector();

    /*!
     * \brief Process one frame and update detected obstacle state
     * Call this once per control loop tick.
     */
    void Update();

    /*!
     * \brief Check if an obstacle is currently detected
     */
    bool IsObstacleDetected() const;

    /*!
     * \brief Get detected obstacle position relative to UAV body frame
     * \param pos Output position (x=forward, y=left, z=up in body frame)
     */
    void GetObstaclePosition(flair::core::Vector3Df &pos) const;

    /*!
     * \brief Get estimated obstacle radius
     */
    float GetObstacleRadius() const;

    /*!
     * \brief Check if detection is enabled via GUI
     */
    bool IsEnabled() const;

private:
    // GUI widgets
    flair::gui::GroupBox *settings_box_;
    flair::gui::ComboBox *enabled_mode_;
    flair::gui::ComboBox *marker_color_;
    flair::gui::DoubleSpinBox *marker_size_cm_;

    // Detection state
    bool obstacle_detected_;
    flair::core::Vector3Df obstacle_pos_;  // relative to UAV
    float obstacle_radius_;

    // HSV thresholds per color preset
    struct HSVRange {
        uint8_t h_min, h_max;
        uint8_t s_min, s_max;
        uint8_t v_min, v_max;
    };

    HSVRange GetColorRange() const;

    // Simple inline HSV conversion for a single RGB pixel
    static void RGBtoHSV(uint8_t r, uint8_t g, uint8_t b,
                          uint8_t &h, uint8_t &s, uint8_t &v);
};

#endif // CAMERA_OBSTACLE_DETECTOR_H
