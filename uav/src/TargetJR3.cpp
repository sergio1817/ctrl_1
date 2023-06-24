// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2015/03/30
//  filename:   TargetJR3.cpp
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
#include "TargetJR3.h"
#include <TabWidget.h>
#include <Tab.h>
#include <FrameworkManager.h>
#include <Matrix.h>
#include <cmath> //for sqrt
#include <Vector3D.h>
#include <DataPlot1D.h>
#include <SpinBox.h>
#include <PushButton.h>
#include <GroupBox.h>
#include <Label.h>

#include <cstring>
#include <string>

using namespace flair::core;
using namespace flair::gui;
using std::string;

namespace flair {
namespace sensor {

TargetJR3::TargetJR3(string name,uint8_t priority) : Thread(getFrameworkManager(),name,priority), IODevice(getFrameworkManager(),name) {
    main_tab=new Tab(getFrameworkManager()->GetTabWidget(),name);
    tab = new TabWidget(main_tab->NewRow(),name);
    setup_tab=new Tab(tab,"Reglages");

    SetPeriodMS(50);
    GroupBox *setup = new GroupBox(setup_tab->NewRow(), "setup");
    T = new SpinBox(setup->NewRow(), "Sampling time", " us", 0, 100000, 1,0);
    setTs = new PushButton(setup->LastRowLastCol(),"Set Ts");
    l2 = new Label(setup->LastRowLastCol(), "Latencia");

}

TargetJR3::~TargetJR3() {
  SafeStop();
  Join();
}

std::string TargetJR3::GetAxisName(unsigned int axisId) const {
  return string("axis") + std::to_string(axisId);
}


void TargetJR3::DrawUserInterface(){
    Tab *plotTab = new Tab(tab, "Measures");
    axisPlot = new DataPlot1D *[axisNumber];
    for (unsigned int i = 0; i < axisNumber; i++) {
        // Start a new row or add up to the current row? We try to keep a 4/3 ratio
        unsigned int columns = sqrtf(16.0 * axisNumber / 9.0);
        LayoutPosition *position;
        if (i % columns == 0) {
            position = plotTab->NewRow();
        } else {
            position = plotTab->LastRowLastCol();
        }
    axisPlot[i] = new DataPlot1D(position,GetAxisName(i),-5,5);
    axis->GetMutex();
    axisPlot[i]->AddCurve(axis->Element(i));
    axis->ReleaseMutex();
    }
}

float TargetJR3::GetAxisValue(unsigned int axisId) const {
  // TODO we'd better throw an exception here
  if (axis == NULL)
    return 0;

  axis->GetMutex();
  float axisValue = axis->Value(axisId, 0);
  axis->ReleaseMutex();
  return axisValue;
}


float TargetJR3::GetFx() const{
    return GetAxisValue(0);
}

float TargetJR3::GetFy() const{
    return GetAxisValue(1);
}

float TargetJR3::GetFz() const{
    return GetAxisValue(2);
}

float TargetJR3::GetMx() const{
    return GetAxisValue(3);
}

float TargetJR3::GetMy() const{
    return GetAxisValue(4);
}

float TargetJR3::GetMz() const{
    return GetAxisValue(5);
}

Vector3Df TargetJR3::GetForce() const{
    return Vector3Df(GetAxisValue(0),GetAxisValue(1),GetAxisValue(2));
}

Vector3Df TargetJR3::GetMoment() const{
    return Vector3Df(GetAxisValue(3),GetAxisValue(4),GetAxisValue(5));
}

void TargetJR3::Run() {


  Message *message;

  //if (getFrameworkManager()->ErrorOccured() || !SensorInitialization()) {
  if (!SensorInitialization()) {
//    SafeStop();
    Thread::Err("An error occured, we don't proceed with the loop.\n");
  } else {
    //axis = new Matrix((IODevice *)this, axisNumber, 1, floatType);
    
    //SetPeriodMS(20); //50Hz
    // AddDataToLog(axis);
    // DrawUserInterface();
    while (!ToBeStopped()) {
      if(setTs->Clicked()) {
        SetPeriodUS((int)T->Value());
      }
      // Thread::Info("Debug: entering acquisition loop\n");
      if (IsDataFrameReady()) {
        // Thread::Info("Debug: data frame is ready\n");

        AcquireSensorData(*axis);
        //l2->SetText("Latecia: %.3f ms",(float)dt_read/1000000);
        // send the data
        axis->SetDataTime(GetTime());
        ProcessUpdate(axis);

        // send pending controller state change request(s)
        while (changeStateQueue.size() != 0) {
          message = changeStateQueue.front();
          if (ProcessMessage(message)) {
            changeStateQueue.pop();
            delete message;
          }
        }
      }
      WaitPeriod();
    }
  }
}

Tab *TargetJR3::GetTab() const { return setup_tab; }

} // end namespace sensor
} // end namespace flair
