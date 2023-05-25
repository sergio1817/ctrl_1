// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2023/05/12
//  filename:   TargetEthJR3.h
//
//  author:     Sergio Urzua
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    class that gets remote controls through an ethernet connection.
//              Typical use case: a remote control is plugged in a workstation
//              and sends remote control
//              data to a distant target (this class) through Wifi
//
//
/*********************************************************************/

#ifndef TARGETETHJR3_H
#define TARGETETHJR3_H

#include "TargetJR3.h"

namespace flair {
namespace core {
class FrameworkManager;
class Matrix;
class TcpSocket;
class UdpSocket;
}
namespace gui {
class Tab;
class TabWidget;
class DataPlot1D;
}
}

namespace flair {
namespace sensor {
/*! \class TargetJR3
*
* \brief Base Class for target side remote controls
*
*/
class TargetEthJR3 : public TargetJR3 {
public:
  TargetEthJR3(std::string name, uint16_t port, uint8_t priority = 0);
  ~TargetEthJR3();
  // void DrawUserInterface();
  std::string GetAxisName(unsigned int axisId) const;
protected:
  bool IsConnected() const;
  // axis stuff
  
  // controller state stuff
  bool ProcessMessage(core::Message *msg);

  bool IsDataFrameReady();
  void AcquireSensorData(core::Matrix &axis);     // responsible for getting the
                                                  // sensor data from the hardware

  bool SensorInitialization();

private:
  std::string name;
  core::TcpSocket *listeningSocket;
  int listeningPort;
  core::TcpSocket *controlSocket = nullptr;
  core::UdpSocket *dataSocket;
  std::string *axisName = nullptr;
  std::string *buttonName = nullptr;
  size_t dataFrameSize;
  char *dataFrameBuffer;
  char *receiveFrameBuffer;
  std::string dataFrameBufferString;
  uint8_t buttonOffset;
};
}
}

#endif // TARGETETHJR3_H
