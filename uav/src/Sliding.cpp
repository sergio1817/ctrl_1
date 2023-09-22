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
#include <Label.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

namespace flair {
namespace filter {

Sliding::Sliding(const LayoutPosition *position, string name): ControlLaw(position->getLayout(), name, 4){
    first_update = true;
    // init matrix
    input = new Matrix(this, 5, 4, floatType, name);
  
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
    GroupBox *pdt = new GroupBox(reglages_groupbox->NewRow(), "PD Thrust");
    GroupBox *ori = new GroupBox(reglages_groupbox->NewRow(), "Attitude");
    GroupBox *mot = new GroupBox(reglages_groupbox->NewRow(), "Motors");

    T = new DoubleSpinBox(pdt->NewRow(), "period, 0 for auto", " s", 0, 1, 0.001,3);
    k1 = new DoubleSpinBox(pdt->NewRow(), "k1:", 0, 5000, 0.1, 3);
    k2 = new DoubleSpinBox(pdt->LastRowLastCol(), "k2:", 0, 5000, 0.1, 3);
    gamma_roll = new DoubleSpinBox(ori->NewRow(), "gamma_roll:", 0, 500, 0.001, 3);
    gamma_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "gamma_pitch:", 0, 500, 0.001, 3);
    gamma_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "gamma_yaw:", 0, 500, 0.001, 3);
    alpha_roll = new DoubleSpinBox(ori->NewRow(), "alpha_roll:", 0, 50000, 0.5, 3);
    alpha_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "alpha_pitch:", 0, 50000, 0.5, 3);
    alpha_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "alpha_yaw:", 0, 50000, 0.5, 3);
    k = new DoubleSpinBox(ori->NewRow(), "k:", 0, 50000, 0.5, 3);
    p = new DoubleSpinBox(ori->LastRowLastCol(), "p:", 0, 50000, 1, 3);
    lo = new Label(ori->LastRowLastCol(), "Latencia ori:");
    Kd_roll = new DoubleSpinBox(ori->NewRow(), "Kd_roll:", 0, 50000, 0.5, 3);
    Kd_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "Kd_pitch:", 0, 50000, 0.5, 3);
    Kd_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "Kd_yaw:", 0, 50000, 0.5, 3);
    sat_r = new DoubleSpinBox(mot->NewRow(), "sat roll:", 0, 1, 0.1);
    sat_p = new DoubleSpinBox(mot->LastRowLastCol(), "sat pitch:", 0, 1, 0.1);
    sat_y = new DoubleSpinBox(mot->LastRowLastCol(), "sat yaw:", 0, 1, 0.1);
    sat_t = new DoubleSpinBox(mot->LastRowLastCol(), "sat thrust:", 0, 1, 0.1);
    
    km = new DoubleSpinBox(mot->LastRowLastCol(), "km:", -10, 10, 0.01, 3);
    
    m = new DoubleSpinBox(pdt->NewRow(),"m",0,2000,0.001,3);
    g = new DoubleSpinBox(pdt->LastRowLastCol(),"g",-10,10,0.01,3);

    AddDataToLog(state);
    
    //GroupBox *c_fisicas = new GroupBox(position->NewRow(), "Constantes Fisicas");
    
    t0 = double(GetTime())/1000000000;
    
    sgnori_p << 0,0,0;
    sgnori << 0,0,0;
    
    
}

Sliding::~Sliding(void) {}

void Sliding::Reset(void) {
    first_update = true;
    t0 = 0;
    t0 = double(GetTime())/1000000000;
    sgnori_p << 0,0,0;
    sgnori << 0,0,0;
//    pimpl_->i = 0;
//    pimpl_->first_update = true;
}

void Sliding::SetValues(float ze, float zp, Vector3Df w, Vector3Df wd, Quaternion q, Quaternion qd){
  input->SetValue(0, 0, ze);
  input->SetValue(1, 0, w.x);
  input->SetValue(2, 0, w.y);
  input->SetValue(3, 0, w.z);
  input->SetValue(4, 0, zp);
  
  input->SetValue(0, 1, q.q0);
  input->SetValue(1, 1, q.q1);
  input->SetValue(2, 1, q.q2);
  input->SetValue(3, 1, q.q3);
  
  input->SetValue(0, 2, qd.q0);
  input->SetValue(1, 2, qd.q1);
  input->SetValue(2, 2, qd.q2);
  input->SetValue(3, 2, qd.q3);

  input->SetValue(0, 3, wd.x);
  input->SetValue(1, 3, wd.y);
  input->SetValue(2, 3, wd.z);
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
    DataPlot1D *Sq = new DataPlot1D(position, "nu_q", -5, 5);
    Sq->AddCurve(state->Element(4), DataPlot::Green);
    Sq->AddCurve(state->Element(5), DataPlot::Red);
    Sq->AddCurve(state->Element(6), DataPlot::Black);
    
}


void Sliding::UpdateFrom(const io_data *data) {
    float tactual=double(GetTime())/1000000000-t0;
    float Trs=0, tau_roll=0, tau_pitch=0, tau_yaw=0, Tr=0;

    //printf("tactual: %f\n",tactual);
    
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

    Eigen::Vector3f w(input->ValueNoMutex(1, 0),input->ValueNoMutex(2, 0),input->ValueNoMutex(3, 0));
    Eigen::Vector3f wd(input->ValueNoMutex(0, 3),input->ValueNoMutex(1, 3),input->ValueNoMutex(2, 3));

    Eigen::Quaternionf q(input->ValueNoMutex(0, 1),input->ValueNoMutex(1, 1),input->ValueNoMutex(2, 1),input->ValueNoMutex(3, 1));
    Eigen::Quaternionf qd(input->ValueNoMutex(0, 2),input->ValueNoMutex(1, 2),input->ValueNoMutex(2, 2),input->ValueNoMutex(3, 2));

    float ze = input->ValueNoMutex(0, 0);
    float zp = input->ValueNoMutex(4, 0);


    Quaternion q2 = Quaternion(input->ValueNoMutex(0, 1),input->ValueNoMutex(1, 1),input->ValueNoMutex(2, 1),input->ValueNoMutex(3, 1));

    input->ReleaseMutex();
    
    Euler currentAngles = q2.ToEuler();


    //Eigen::Vector3f alphao_v(alpha_roll->Value(), alpha_pitch->Value(), alpha_yaw->Value());
    Eigen::Matrix3f alphao = Eigen::Vector3f(alpha_roll->Value(), alpha_pitch->Value(), alpha_yaw->Value()).asDiagonal();

    //Eigen::Vector3f gammao_v(gamma_roll->Value(), gamma_pitch->Value(), gamma_yaw->Value());
    Eigen::Matrix3f gammao = Eigen::Vector3f(gamma_roll->Value(), gamma_pitch->Value(), gamma_yaw->Value()).asDiagonal();

    //Eigen::Vector3f Kdv(Kd_roll->Value(), Kd_pitch->Value(), Kd_yaw->Value());
    Eigen::Matrix3f Kdm = Eigen::Vector3f(Kd_roll->Value(), Kd_pitch->Value(), Kd_yaw->Value()).asDiagonal();

    Eigen::Quaternionf qe = q*qd.conjugate();

    flair::core::Time t0_o = GetTime();

    Eigen::Vector3f we = w -wd;

    Eigen::Vector3f QdTqe3 = qd.toRotationMatrix().transpose()*qe.vec();

    Eigen::Vector3f nu = we + alphao*QdTqe3;
    
    Eigen::Vector3f nu_t0 = 0.1*Eigen::Vector3f(1,1,1);
    
    Eigen::Vector3f nud = nu_t0*exp(-k->Value()*(tactual));
    
    Eigen::Vector3f nuq = nu-nud;

    sgnori_p = signth(nuq,p->Value());
    sgnori = rk4_vec(sgnori, sgnori_p, delta_t);

    Eigen::Vector3f nur = nuq + gammao*sgnori;

    Eigen::Vector3f tau = -Kdm*nur;

    flair::core::Time dt_ori = GetTime() - t0_o;

    //lo->SetText("Latecia ori: %.3f ms",(float)dt_ori/1000000);
    
    Trs =  (m->Value()*(k1->Value()*zp + k2->Value()*ze + g->Value()))/(cosf(currentAngles.pitch)*cosf(currentAngles.roll));
    
    tau_roll = (float)tau(0)/km->Value();
    
    tau_pitch = (float)tau(1)/km->Value();
    
    tau_yaw = (float)tau(2)/km->Value();
    
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
    state->SetValueNoMutex(4, 0, nuq(0));
    state->SetValueNoMutex(5, 0, nuq(1));
    state->SetValueNoMutex(6, 0, nuq(2));
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
