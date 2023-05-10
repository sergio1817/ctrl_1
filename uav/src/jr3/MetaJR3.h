// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file MetaJR3.h
 * \brief Classe intégrant la manette DualShock3 et les consignes joystick
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2014/01/14
 * \version 3.4
 */

#ifndef MetaJR3_H
#define MetaJR3_H

#include "TargetJR3.h"
#include <IODevice.h>
#include <Vector3D.h>

namespace flair {
namespace core {
class AhrsData;
class Quaternion;
}
namespace filter {
class Ahrs;
}
namespace gui {
class Tab;
class TabWidget;
class DataPlot1D;
}
}

namespace flair {
namespace meta {

/*! \class MetaJR3
*
* \brief Classe intégrant la manette MetaJR3
*/
class MetaJR3 : public core::IODevice {

public:
  MetaJR3(std::string name, sensor::TargetJR3 *jr3);
  ~MetaJR3();

    void UpdateFrom(const flair::core::io_data *data);
    void DrawUserInterface();
  
    gui::TabWidget *tab;

    gui::DataPlot1D **axisPlot;

    int axisNumber;

    float GetFx() const;
    float GetFy() const;
    float GetFz() const;
    float GetMx() const;
    float GetMy() const;
    float GetMz() const;

    flair::core::Vector3Df GetForce() const;
    flair::core::Vector3Df GetMoment() const;

    

        private:
            //void UpdateFrom(const flair::core::io_data *data);
            sensor::TargetJR3 *jr3;
            flair::core::Matrix *data_s;
    };
} // end namespace meta
} // end namespace flair
#endif // MetaJR3_H
