// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2015/04/14
//  filename:   TargetJR3.h
//
//  author:     Gildas Bayard
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Base class for target side remote controls
//
//
/*********************************************************************/

#ifndef TargetJR3_H
#define TargetJR3_H

#include <IODevice.h>
#include <Thread.h>
#include <stdint.h>
#include <Vector3D.h>
#include <queue>

namespace flair {
namespace core {
class FrameworkManager;
class Matrix;
class Socket;
class io_data;
}
namespace gui {
class Tab;
class TabWidget;
class DataPlot1D;
class PushButton;
class SpinBox;
}
}

namespace flair {
namespace sensor {
enum class ControllerAction;

/*! \class TargetJR3
*
* \brief Base Class for target side remote controls
*
*/
class TargetJR3 : public core::Thread, public core::IODevice {
public:
  TargetJR3(std::string name,uint8_t priority = 0);
  ~TargetJR3();
  // void DrawUserInterface();
  virtual bool IsConnected() const = 0;
  virtual bool IsDataFrameReady() = 0;
  // axis stuff
  unsigned int GetAxisNumber() const;
  virtual std::string GetAxisName(unsigned int axisId) const;
  float
  GetAxisValue(unsigned int axisId) const; 
  // controller state stuff
  
  void UpdateFrom(const core::io_data *data){}; // TODO
  void DrawUserInterface();
  gui::Tab *GetTab(void) const;
  gui::TabWidget *tab;
  flair::gui::PushButton *setTs;
  gui::SpinBox *T;

    float GetFx() const;
    float GetFy() const;
    float GetFz() const;
    float GetMx() const;
    float GetMy() const;
    float GetMz() const;

    flair::core::Vector3Df GetForce() const;
    flair::core::Vector3Df GetMoment() const;

    core::Matrix *axis = nullptr;

protected:
  virtual bool ProcessMessage(core::Message *msg) = 0;
  void QueueMessage(core::Message msg);
  virtual bool SensorInitialization() = 0; // {return true;};
  // axis stuff
  unsigned int axisNumber;
  gui::DataPlot1D **axisPlot;
  
  virtual void AcquireSensorData(core::Matrix &axis) = 0; // responsible for
                                                          // getting the axis
                                                          // data from the
                                                          // hardware
  uint16_t bitsPerAxis;
  // button stuff
  unsigned int buttonNumber;
  // controller state stuff
  unsigned int ledNumber;

private:
  void Run();
  std::queue<core::Message *> changeStateQueue;
  flair::gui::Tab *main_tab;
  flair::gui::Tab *setup_tab;
};
}
}

#endif // TargetJR3_H
