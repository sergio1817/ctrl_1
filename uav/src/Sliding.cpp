// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2011/05/01
//  filename:   Pid_impl.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a PID
//
//
/*********************************************************************/
#include "Sliding.h"
#include "NMethods.h"
#include <Matrix.h>
#include <Vector3D.h>
#include <Quaternion.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <cmath>
#include <Euler.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

namespace flair {
namespace filter {

Sliding::Sliding(const LayoutPosition *position, string name): ControlLaw(position->getLayout(), name, 4){
    first_update = true;
    // init matrix
    input = new Matrix(this, 5, 3, floatType, name);
  
    MatrixDescriptor *desc = new MatrixDescriptor(7, 1);
    desc->SetElementName(0, 0, "u_roll");
    desc->SetElementName(1, 0, "u_pitch");
    desc->SetElementName(2, 0, "u_yaw");
    desc->SetElementName(3, 0, "u_z");
    desc->SetElementName(4, 0, "nur_roll");
    desc->SetElementName(5, 0, "nur_pitch");
    desc->SetElementName(6, 0, "nur_yaw");
    state = new Matrix(this, desc, floatType, name);
    delete desc;


    GroupBox *reglages_groupbox = new GroupBox(position, name);
    T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s", 0, 1, 0.001,3);
    k1 = new DoubleSpinBox(reglages_groupbox->NewRow(), "k1:", 0, 5000, 0.1, 3);
    k2 = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "k2:", 0, 5000, 0.1, 3);
    gamma = new DoubleSpinBox(reglages_groupbox->NewRow(), "gamma:", -500, 500, 0.0001, 3);
    p = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "p:", 0, 50000, 1, 3);
    alpha = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "alpha:", 0, 50000, 0.5, 3);
    k = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "k:", 0, 50000, 0.5, 3);
    Kd = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "Kd:", 0, 50000, 0.5, 3);
    sat_r = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat roll:", 0, 1, 0.1);
    sat_p = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat pitch:", 0, 1, 0.1);
    sat_y = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat yaw:", 0, 1, 0.1);
    sat_t = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat thrust:", 0, 1, 0.1);
    
    km = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "km:", -10, 10, 0.01, 3);
    
    m = new DoubleSpinBox(reglages_groupbox->NewRow(),"m",0,2000,0.001,3);
    g = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"g",-10,10,0.01,3);
    
    //GroupBox *c_fisicas = new GroupBox(position->NewRow(), "Constantes Fisicas");
    
    t0 = double(GetTime())/1000000000;
    
    sgnp = Vector3Df(0,0,0);
    sgn = Vector3Df(0,0,0);
    
    
}

Sliding::~Sliding(void) {}

void Sliding::Reset(void) {
    first_update = true;
    t0 = 0;
    t0 = double(GetTime())/1000000000;
    sgnp = Vector3Df(0,0,0);
    sgn = Vector3Df(0,0,0);
//    pimpl_->i = 0;
//    pimpl_->first_update = true;
}

void Sliding::SetValues(float ze, float zp, Vector3Df we, Quaternion q, Quaternion qd){
  input->SetValue(0, 0, ze);
  input->SetValue(1, 0, we.x);
  input->SetValue(2, 0, we.y);
  input->SetValue(3, 0, we.z);
  input->SetValue(4, 0, zp);
  
  input->SetValue(0, 1, q.q0);
  input->SetValue(1, 1, q.q1);
  input->SetValue(2, 1, q.q2);
  input->SetValue(3, 1, q.q3);
  
  input->SetValue(0, 2, qd.q0);
  input->SetValue(1, 2, qd.q1);
  input->SetValue(2, 2, qd.q2);
  input->SetValue(3, 2, qd.q3);
}

void Sliding::UseDefaultPlot(const LayoutPosition *position) {
    DataPlot1D *rollg = new DataPlot1D(position, "u_roll", -1, 1);
    rollg->AddCurve(state->Element(0));
    
}

void Sliding::UseDefaultPlot2(const LayoutPosition *position) {
    DataPlot1D *pitchg = new DataPlot1D(position, "u_pitch", -1, 1);
    pitchg->AddCurve(state->Element(1));
    
}

void Sliding::UseDefaultPlot3(const LayoutPosition *position) {
    DataPlot1D *yawg = new DataPlot1D(position, "u_yaw", -1, 1);
    yawg->AddCurve(state->Element(2));
    
}

void Sliding::UseDefaultPlot4(const LayoutPosition *position) {    
    DataPlot1D *uz = new DataPlot1D(position, "u_z", -1, 1);
    uz->AddCurve(state->Element(3));
    
}

void Sliding::UseDefaultPlot5(const LayoutPosition *position) {    
    DataPlot1D *Sq = new DataPlot1D(position, "nu_r", -5, 5);
    Sq->AddCurve(state->Element(4), DataPlot::Green);
    Sq->AddCurve(state->Element(5), DataPlot::Red);
    Sq->AddCurve(state->Element(6), DataPlot::Black);
    
}


void Sliding::UpdateFrom(const io_data *data) {
    float tactual=double(GetTime())/1000000000-t0;
    float Trs=0, tau_roll=0, tau_pitch=0, tau_yaw=0, Tr=0;

    printf("tactual: %f\n",tactual);
    
    if (T->Value() == 0) {
        delta_t = (float)(data->DataDeltaTime()) / 1000000000.;
    } else {
        delta_t = T->Value();
    }
    
    if (first_update == true) {
        delta_t = 0;
        first_update = false;
    }
    
    const Matrix* input = dynamic_cast<const Matrix*>(data);
  
    if (!input) {
        Warn("casting %s to Matrix failed\n",data->ObjectName().c_str(),TIME_INFINITE);
        return;
    }


    input->GetMutex();

    float ze = input->ValueNoMutex(0, 0);
    float zp = input->ValueNoMutex(4, 0);
    
    Vector3Df we = Vector3Df(input->ValueNoMutex(1, 0),input->ValueNoMutex(2, 0),input->ValueNoMutex(3, 0));

    Quaternion q = Quaternion(input->ValueNoMutex(0, 1),input->ValueNoMutex(1, 1),input->ValueNoMutex(2, 1),input->ValueNoMutex(3, 1));
    Quaternion qd = Quaternion(input->ValueNoMutex(0, 2),input->ValueNoMutex(1, 2),input->ValueNoMutex(2, 2),input->ValueNoMutex(3, 2));
    
    input->ReleaseMutex();
    
    Euler currentAngles = q.ToEuler();
    
    Quaternion qdc = qd.GetConjugate();
    Quaternion qe = q*qdc;
    
    Quaternion QdTqe = qdc*qe*qd;
    Vector3Df QdTqe3 = Vector3Df(QdTqe.q1,QdTqe.q2,QdTqe.q3);
    
    Vector3Df nu = we + alpha->Value()*QdTqe3;
    
    Vector3Df nu_t0 = 0.1*Vector3Df(1,1,1);
    
    Vector3Df nud = nu_t0*exp(-k->Value()*(tactual));
    
    Vector3Df nuq = nu-nud;
    
    sgnp.x = signth(nuq.x,p->Value());
    sgnp.y = signth(nuq.y,p->Value());
    sgnp.z = signth(nuq.z,p->Value());
    
    sgn.x = rk4(function1d, sgn.x, sgnp.x, delta_t);
    sgn.y = rk4(function1d, sgn.y, sgnp.y, delta_t);
    sgn.z = rk4(function1d, sgn.z, sgnp.z, delta_t);
    
    Vector3Df nur = nuq + gamma->Value()*sgn;
    
    Vector3Df tau = -Kd->Value()*nur;
    
    Trs =  (m->Value()*(k1->Value()*zp + k2->Value()*ze + g->Value()))/(cosf(currentAngles.pitch)*cosf(currentAngles.roll));
    
    tau_roll = (float)tau.x/km->Value();
    
    tau_pitch = (float)tau.y/km->Value();
    
    tau_yaw = (float)tau.z/km->Value();
    
    Tr = (float)Trs/km->Value();
    
    tau_roll = -Sat(tau_roll,sat_r->Value());
    tau_pitch = -Sat(tau_pitch,sat_p->Value());
    tau_yaw = -Sat(tau_yaw,sat_y->Value());
    Tr = Sat(Tr,sat_t->Value());
    
    state->GetMutex();
    state->SetValueNoMutex(0, 0, tau_roll);
    state->SetValueNoMutex(1, 0, tau_pitch);
    state->SetValueNoMutex(2, 0, tau_yaw);
    state->SetValueNoMutex(3, 0, Tr);
    state->SetValueNoMutex(4, 0, nur.x);
    state->SetValueNoMutex(5, 0, nur.y);
    state->SetValueNoMutex(6, 0, nur.z);
    state->ReleaseMutex();


    output->SetValue(0, 0, tau_roll);
    output->SetValue(1, 0, tau_pitch);
    output->SetValue(2, 0, tau_yaw);
    output->SetValue(3, 0, Tr);
    output->SetDataTime(data->DataTime());
    
    ProcessUpdate(output);
    
}

float Sliding::Sat(float value, float borne) {
  if (value < -borne)
    return -borne;
  if (value > borne)
    return borne;
  return value;
}


} // end namespace filter
} // end namespace flair
