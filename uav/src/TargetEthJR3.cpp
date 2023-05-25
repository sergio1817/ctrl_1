// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2015/03/30
//  filename:   TargetEthJR3.cpp
//
//  author:     Gildas Bayard
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    class that gets remote controls through an ethernet connection.
//              Typical use case: a remote control is plugged in a workstation
//              and sends remote control
//              data to a distant target (this class) through Wifi//
//
/*********************************************************************/
#include "TargetEthJR3.h"
#include <FrameworkManager.h>
#include <TcpSocket.h>
#include <UdpSocket.h>
#include <cstring>
#include <string>
#include <Matrix.h>
#include <stdexcept>
#include <SpinBox.h>
#include <PushButton.h>
#include <bits/stdc++.h>  

using namespace flair::core;
using namespace flair::gui;
using std::string;

namespace flair {
namespace sensor {
    enum class SensorrAction {Exit};

TargetEthJR3::TargetEthJR3(string name, uint16_t _port, uint8_t priority): TargetJR3(name, priority), listeningPort(_port), name(name) {
    const bool blocking = true;
    listeningSocket = new TcpSocket(getFrameworkManager(), "TEJR3_listening_socket", blocking, blocking);
    dataSocket = new UdpSocket(getFrameworkManager(), "TEJR3_data_socket", _port + 1); // receiving side
    
}

TargetEthJR3::~TargetEthJR3() {
  // We are (currently) the server side. We must ask the client side to initiate
  // tcp connexion closing to avoid the server socket
  // to get stuck in TIME_WAIT state
  Message msg(32);
  if (controlSocket) {
    Message cancelAcquisition(sizeof(SensorrAction));
    SensorrAction exit = SensorrAction::Exit;
    memcpy(cancelAcquisition.buffer, &exit, sizeof(SensorrAction));
    controlSocket->SendMessage(cancelAcquisition.buffer, cancelAcquisition.bufferSize);
    // We don't expect any more data from the client, we're just waiting for the
    // socket to be closed by the client
    controlSocket->RecvMessage(msg.buffer, msg.bufferSize);
    delete controlSocket;
  }

  // TargetJR3 calls TargetEthJR3 methods in its run
  // we must stop the thread now
  SafeStop();
  Join();
  delete dataSocket;
  delete listeningSocket;
}

bool TargetEthJR3::IsConnected() const {
  // TODO
}

bool TargetEthJR3::IsDataFrameReady() {
  // read up to the last data
  if(setTs->Clicked()) {
      SetPeriodUS((int)T->Value());
  }

  ssize_t received;
  bool fullDatagramReceived = false;

    do {
        received=dataSocket->RecvMessage(receiveFrameBuffer,64,TIME_NONBLOCK);
        if (received>0) {
            //a full datagram has been read in receiveFrameBuffer
            fullDatagramReceived=true;
            //we swap the data and reception buffers to avoid copy
            char *swapFrameBuffer=dataFrameBuffer;
            dataFrameBuffer=receiveFrameBuffer;
            receiveFrameBuffer=swapFrameBuffer;
        }
    } while (!(received<0));
    dataFrameBufferString = dataFrameBuffer;
  return fullDatagramReceived;
}

void TargetEthJR3::AcquireSensorData(core::Matrix &axis) {
    string line;
    std::stringstream S(dataFrameBufferString); 
    unsigned int i = 0;
    flair::core::Time t0 = GetTime();
    axis.GetMutex();
    // for (unsigned int i = 0; i < axisNumber; i++) {
    //     axis.SetValueNoMutex(i, 0, );
    // }
    while (getline(S, line, ';')){
        axis.SetValueNoMutex(i, 0, std::stof(line));
        i++;
        if(i == axisNumber){
            break;
        }
    }
    axis.ReleaseMutex();
    dt_read = GetTime() - t0;
}


string TargetEthJR3::GetAxisName(unsigned int axisId) const {
  // TODO: should throw an exception if axisName==NULL or axisId>axisNumber
  return axisName[axisId];
}


bool TargetEthJR3::ProcessMessage(Message *msg) {
  return !(controlSocket->SendMessage(msg->buffer, msg->bufferSize, TIME_INFINITE) < 0);
}

bool TargetEthJR3::SensorInitialization() {
  char message[1024];
  ssize_t received;
  bool connected = false;
  bool mustReadMore;

  listeningSocket->Listen(listeningPort);
  Thread::Info("Debug: Listening to port %d\n", listeningPort);
  // accept incoming connection
  while (!controlSocket) {
    try {
      controlSocket = listeningSocket->Accept(100000000);
    } catch (std::logic_error &e) {
      Thread::Err("%s\n",e.what());
      if (ToBeStopped())
        return false;
    } catch (std::runtime_error e) {
      // timeout
      //Thread::Err("%s\n",e.what());
      if (ToBeStopped())
        return false;
    }
  }
  Thread::Info("Debug: Connexion accepted\n");

  // get axis stuff
  bool axisNumberRead = false;
  while (!axisNumberRead) {
    try {
      axisNumber = controlSocket->ReadUInt32(100000000);
      // Thread::Info("Debug: axisNumber %d\n", axisNumber);
      axisNumberRead = true;
    } catch (std::runtime_error e) {
      // timeout
      if (ToBeStopped())
        return false;
    }
  }
  MatrixDescriptor *desc = new MatrixDescriptor(axisNumber, 1);
  bool bitsPerAxisRead = false;
  while (!bitsPerAxisRead) {
    try {
      bitsPerAxis = controlSocket->ReadUInt32(100000000);
      // Thread::Info("Debug: bits per axis %d\n", bitsPerAxis);
      bitsPerAxisRead = true;
    } catch (std::runtime_error e) {
      // timeout
      if (ToBeStopped())
        return false;
    }
  }
  axisName = new string[axisNumber];
  for (unsigned int i = 0; i < axisNumber; i++) {
    // read string size
    int stringSize;
    bool stringSizeRead = false;
    while (!stringSizeRead) {
      try {
        stringSize = controlSocket->ReadUInt32(100000000);
        stringSizeRead = true;
      } catch (std::runtime_error e) {
        // timeout
        if (ToBeStopped())
          return false;
      }
    }
    // read string
    bool axisNameRead = false;
    while (!axisNameRead) {
      try {
        axisName[i] = controlSocket->ReadString(stringSize, 100000000);
        axisNameRead = true;
      } catch (std::runtime_error e) {
        // timeout
        if (ToBeStopped())
          return false;
      }
    }
    
    desc->SetElementName(i, 0, axisName[i]);
    
    //Thread::Info("Debug: axisName for axis %d %s\n", i, axisName[i].c_str());
    
  }

  axis = new Matrix((IODevice *)this, desc, floatType, name);
  //    dataFrameSize=axisNumber*sizeof(float)+buttonNumber/8*sizeof(uint8_t);
//   buttonOffset = (axisNumber * bitsPerAxis) / 8;
//   if ((axisNumber * bitsPerAxis) % 8 != 0)
//     buttonOffset++;
//   dataFrameSize = buttonOffset + (buttonNumber / 8) * sizeof(uint8_t);
//   if ((buttonNumber % 8) != 0)
//     dataFrameSize++;

  AddDataToLog(axis);
  DrawUserInterface();

  dataFrameBuffer = new char[64];
  receiveFrameBuffer = new char[64];

  Thread::Info("JR3 connected with host side\n");
  return true;
}

} // end namespace sensor
} // end namespace flair
