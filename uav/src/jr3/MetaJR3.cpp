// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2014/01/14
//  filename:   MetaJR3.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    objet integrant la manette DualShock3 et les consignes joystick
//
//
/*********************************************************************/

#include "MetaJR3.h"
#include "TargetJR3.h"
#include <Tab.h>
#include <TabWidget.h>
#include <FrameworkManager.h>
#include <Vector3D.h>
#include <Matrix.h>
#include <cmath> //for sqrt
#include <DataPlot1D.h>

using std::string;
using namespace flair::core;
using namespace flair::filter;
using namespace flair::gui;
using namespace flair::sensor;

namespace flair {
namespace meta {

MetaJR3::MetaJR3(string name, TargetJR3 *jr3): jr3(jr3),IODevice((IODevice*)jr3, name) {
  //getFrameworkManager()->AddDeviceToLog(pimpl_->joy_ref);
  //this->self = self;
    jr3->SetPeriodMS(50);
    jr3->Start();
    tab = jr3->tab;

    data_s = new Matrix(this, 6, 1, floatType, name);


    axisNumber = jr3->GetAxisNumber();

    SetIsReady(true);
}

MetaJR3::~MetaJR3() { delete jr3; }


void MetaJR3::DrawUserInterface() {
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
    axisPlot[i] = new DataPlot1D(position,jr3->GetAxisName(i),-5,5);
    axisPlot[i]->AddCurve(data_s->Element(i));
    }
}


float MetaJR3::GetFx() const{
    return jr3->GetAxisValue(0);
}

float MetaJR3::GetFy() const{
    return jr3->GetAxisValue(1);
}

float MetaJR3::GetFz() const{
    return jr3->GetAxisValue(2);
}

float MetaJR3::GetMx() const{
    return jr3->GetAxisValue(3);
}

float MetaJR3::GetMy() const{
    return jr3->GetAxisValue(4);
}

float MetaJR3::GetMz() const{
    return jr3->GetAxisValue(5);
}

Vector3Df MetaJR3::GetForce() const{
    return Vector3Df(jr3->GetAxisValue(0),jr3->GetAxisValue(1),jr3->GetAxisValue(2));
}

Vector3Df MetaJR3::GetMoment() const{
    return Vector3Df(jr3->GetAxisValue(3),jr3->GetAxisValue(4),jr3->GetAxisValue(5));
}


void MetaJR3::UpdateFrom(const io_data *data) {
  const Matrix* input = dynamic_cast<const Matrix*>(data);
  
  if (!input) {
      this->Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
      return;
  }

  // on prend une fois pour toute le mutex et on fait des accès directs
  input->GetMutex();

  

  if (!getFrameworkManager()->ConnectionLost()) {
    input->GetMutex();
    data_s->SetValue(0,0,input->ValueNoMutex(0, 0));
    data_s->SetValue(1,0,input->ValueNoMutex(1, 0));
    data_s->SetValue(2,0,input->ValueNoMutex(2, 0));
    data_s->SetValue(3,0,input->ValueNoMutex(3, 0));
    data_s->SetValue(4,0,input->ValueNoMutex(4, 0));
    data_s->SetValue(5,0,input->ValueNoMutex(5, 0));

    input->ReleaseMutex();
    }else{
    data_s->SetValue(0,0,0);
    data_s->SetValue(1,0,0);
    data_s->SetValue(2,0,0);
    data_s->SetValue(3,0,0);
    data_s->SetValue(4,0,0);
    data_s->SetValue(5,0,0);
  }
  input->ReleaseMutex();


}


} // end namespace meta
} // end namespace flair
