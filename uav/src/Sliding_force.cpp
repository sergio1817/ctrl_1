// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2023/05/25
//  filename:   Sliding_force.cpp
//
//  author:     Sergio Urzua
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a force-position sliding mode controller
//
//
/*********************************************************************/
#include "Sliding_force.h"
#include "NMethods.h"
#include <Matrix.h>
#include <Vector3D.h>
#include <Eigen/Dense>
#include <TabWidget.h>
#include <CheckBox.h>
#include <Quaternion.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <cmath>
#include <Euler.h>
#include <iostream>
#include <Label.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

namespace flair {
namespace filter {

Sliding_force::Sliding_force(const LayoutPosition *position, string name): ControlLaw(position->getLayout(), name, 4){ // Salidas 4
    first_update = true;
    // init matrix
    input = new Matrix(this, 4, 10, floatType, name);

    MatrixDescriptor *desc = new MatrixDescriptor(39, 1);
    desc->SetElementName(0, 0, "u_roll");
    desc->SetElementName(1, 0, "u_pitch");
    desc->SetElementName(2, 0, "u_yaw");
    desc->SetElementName(3, 0, "u_z");
    desc->SetElementName(4, 0, "roll_d");
    desc->SetElementName(5, 0, "pitch_d");
    desc->SetElementName(6, 0, "yaw_d");
    desc->SetElementName(7, 0, "Sp_x");
    desc->SetElementName(8, 0, "Sp_y");
    desc->SetElementName(9, 0, "Sp_z");
    desc->SetElementName(10, 0, "Sa_roll");
    desc->SetElementName(11, 0, "Sa_pitch");
    desc->SetElementName(12, 0, "Sa_yaw");
    desc->SetElementName(13, 0, "Sf_x");
    desc->SetElementName(14, 0, "Svf_y");
    desc->SetElementName(15, 0, "Svf_z");
    desc->SetElementName(16, 0, "Sr_x");
    desc->SetElementName(17, 0, "Sr_y");
    desc->SetElementName(18, 0, "Sr_z");
    desc->SetElementName(19, 0, "Fx");
    desc->SetElementName(20, 0, "Fy");
    desc->SetElementName(21, 0, "Fz");
    desc->SetElementName(22, 0, "Fdx");
    desc->SetElementName(23, 0, "Fdy");
    desc->SetElementName(24, 0, "Fdz");
    desc->SetElementName(25, 0, "pitch_f");
    desc->SetElementName(26, 0, "roll");
    desc->SetElementName(27, 0, "pitch");
    desc->SetElementName(28, 0, "yaw");
    desc->SetElementName(29, 0, "tau_roll");
    desc->SetElementName(30, 0, "tau_pitch");
    desc->SetElementName(31, 0, "tau_yaw");
    desc->SetElementName(32, 0, "u_x");
    desc->SetElementName(33, 0, "u_y");
    desc->SetElementName(34, 0, "u_z");
    desc->SetElementName(35, 0, "T");
    desc->SetElementName(36, 0, "Levant_err_x");
    desc->SetElementName(37, 0, "Levant_err_y");
    desc->SetElementName(38, 0, "Levant_err_z");
    state = new Matrix(this, desc, floatType, name);
    delete desc;


    GroupBox *reglages_groupbox = new GroupBox(position, name);
    GroupBox *num = new GroupBox(reglages_groupbox->NewRow(), "Integral y derivada");
    GroupBox *ori = new GroupBox(reglages_groupbox->NewRow(), "Orientacion");
    GroupBox *pos = new GroupBox(reglages_groupbox->NewRow(), "Posicion");
    GroupBox *forc = new GroupBox(reglages_groupbox->NewRow(), "Fuerza");
    GroupBox *mot = new GroupBox(reglages_groupbox->NewRow(), "Motores");

    T = new DoubleSpinBox(num->NewRow(), "period, 0 for auto", " s", 0, 1, 0.01,3);
    alpha_l1 = new DoubleSpinBox(num->NewRow(), "alpha 1 Levant:", 0, 500, 0.001, 3);
    alpha_l2 = new DoubleSpinBox(num->LastRowLastCol(), "alpha 2 Levant:", 0, 500, 0.001, 3);
    alpha_l3 = new DoubleSpinBox(num->LastRowLastCol(), "alpha 3 Levant:", 0, 500, 0.001, 3);
    lamb_l1 = new DoubleSpinBox(num->NewRow(), "lambda 1 Levant:", 0, 500, 0.001, 3);
    lamb_l2 = new DoubleSpinBox(num->LastRowLastCol(), "lambda 2 Levant:", 0, 500, 0.001, 3);
    lamb_l3 = new DoubleSpinBox(num->LastRowLastCol(), "lambda 3 Levant:", 0, 500, 0.001, 3);
    //levantd = new CheckBox(num->LastRowLastCol(), "Levant");

    gamma_roll = new DoubleSpinBox(ori->NewRow(), "gamma_roll:", 0, 500, 0.001, 3);
    gamma_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "gamma_pitch:", 0, 500, 0.001, 3);
    gamma_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "gamma_yaw:", 0, 500, 0.001, 3);
    alpha_roll = new DoubleSpinBox(ori->NewRow(), "alpha_roll:", 0, 50000, 0.5, 3);
    alpha_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "alpha_pitch:", 0, 50000, 0.5, 3);
    alpha_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "alpha_yaw:", 0, 50000, 0.5, 3);
    k = new DoubleSpinBox(ori->NewRow(), "k:", 0, 50000, 0.5, 3);
    p = new DoubleSpinBox(ori->LastRowLastCol(), "p:", 0, 50000, 1, 3);
    lo = new Label(ori->LastRowLastCol(), "Latencia ori");
    Kd_roll = new DoubleSpinBox(ori->NewRow(), "Kd_rol:", 0, 50000, 0.5, 3);
    Kd_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "Kd_pitch:", 0, 50000, 0.5, 3);
    Kd_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "Kd_yaw:", 0, 50000, 0.5, 3);

    gamma_x = new DoubleSpinBox(pos->NewRow(), "gamma_x:", 0, 500, 0.001, 3);
    gamma_y = new DoubleSpinBox(pos->LastRowLastCol(), "gamma_y:", 0, 500, 0.001, 3);
    gamma_z = new DoubleSpinBox(pos->LastRowLastCol(), "gamma_z:", 0, 500, 0.001, 3);
    alpha_x = new DoubleSpinBox(pos->NewRow(), "alpha_x:", 0, 50000, 0.5, 3);
    alpha_y = new DoubleSpinBox(pos->LastRowLastCol(), "alpha_y:", 0, 50000, 0.5, 3);
    alpha_z = new DoubleSpinBox(pos->LastRowLastCol(), "alpha_z:", 0, 50000, 0.5, 3);
    Kp_x = new DoubleSpinBox(pos->NewRow(), "Kp_x:", 0, 50000, 0.5, 3);
    Kp_y = new DoubleSpinBox(pos->LastRowLastCol(), "Kp_y:", 0, 50000, 0.5, 3);
    Kp_z = new DoubleSpinBox(pos->LastRowLastCol(), "Kp_z:", 0, 50000, 0.5, 3);
    

    //gamma_fx = new DoubleSpinBox(forc->NewRow(), "gamma_fx:", 0, 500, 0.001, 3);
    //kf_x = new DoubleSpinBox(forc->LastRowLastCol(), "kf_x:", -50, 50000, 0.5, 3);
    //alpha_fx = new DoubleSpinBox(forc->LastRowLastCol(), "alpha_fx:", 0, 500, 0.001, 3);
    alphapf1 = new DoubleSpinBox(forc->NewRow(), "alpha_pf:", 0, 500, 0.001, 3);
    gamma_fy = new DoubleSpinBox(forc->NewRow(), "gamma_fy:", 0, 500, 0.001, 3);
    gamma_fz = new DoubleSpinBox(forc->LastRowLastCol(), "gamma_fz:", 0, 500, 0.001, 3);
    
    eta_1 = new DoubleSpinBox(forc->NewRow(), "eta_1:", 0, 50000, 0.5, 3);
    eta_2 = new DoubleSpinBox(forc->LastRowLastCol(), "eta_2:", 0, 50000, 0.5, 3);
    beta_1 = new DoubleSpinBox(forc->LastRowLastCol(), "beta_1:", 0, 50000, 0.5, 3);
    beta_2 = new DoubleSpinBox(forc->LastRowLastCol(), "beta_2:", 0, 50000, 0.5, 3);
    kf = new DoubleSpinBox(forc->NewRow(), "kf:", 0, 50000, 0.5, 3);
    muf = new DoubleSpinBox(forc->LastRowLastCol(), "mu_f:", 0, 50000, 0.5, 3);

    mu_t = new DoubleSpinBox(forc->NewRow(), "mu_t:", 0, 50000, 0.5, 3);

    sat_r = new DoubleSpinBox(mot->NewRow(), "sat roll:", 0, 1, 0.1);
    sat_p = new DoubleSpinBox(mot->LastRowLastCol(), "sat pitch:", 0, 1, 0.1);
    sat_y = new DoubleSpinBox(mot->LastRowLastCol(), "sat yaw:", 0, 1, 0.1);
    sat_t = new DoubleSpinBox(mot->LastRowLastCol(), "sat thrust:", 0, 1, 0.1);
    
    km = new DoubleSpinBox(mot->NewRow(), "km:", -100, 100, 0.01, 3);
    km_z = new DoubleSpinBox(mot->LastRowLastCol(), "km_z:", -100, 100, 0.01, 3);
    
    m = new DoubleSpinBox(pos->NewRow(),"m",0,2000,0.001,3);
    g = new DoubleSpinBox(pos->LastRowLastCol(),"g",0,10,0.01,3);
    mup = new DoubleSpinBox(pos->LastRowLastCol(), "mup:", 0, 50000, 0.5, 3);
    lp = new Label(pos->LastRowLastCol(), "Latencia pos");
    
    t0 = double(GetTime())/1000000000;

    levant = Levant_diff("tanh", 8, 6, 3000);

    sgnpos_p << 0,0,0;
    sgnpos << 0,0,0;

    sgnori_p << 0,0,0;
    sgnori << 0,0,0;

    dLamb << 0,0;
    Sf << 0,0;
    sgnfp << 0,0;
    sgnf << 0,0;
    dFi_ = 0;

    sgnfx = 0;
    sgnfxp = 0;
    
    AddDataToLog(state);
}

Sliding_force::~Sliding_force(void) {}

void Sliding_force::Reset(void) {
    first_update = true;
    t0 = 0;
    t0 = double(GetTime())/1000000000;
    sgnori_p << 0,0,0;
    sgnori << 0,0,0;

    levant.Reset();

    sgnpos_p << 0,0,0;
    sgnpos << 0,0,0;

    dLamb << 0,0;
    Sf << 0,0;
    sgnfp << 0,0;
    sgnf << 0,0;
    dFi_ = 0;

    sgnfx = 0;
    sgnfxp = 0;

    //Printf("Reset\n");
}

void Sliding_force::SetValues(Vector3Df xie, Vector3Df xiep, Vector3Df xid, Vector3Df xidpp, Vector3Df xidppp, Vector3Df w, Quaternion q, Vector3Df F, Vector3Df Fd){


    input->SetValue(0, 0, xie.x);
    input->SetValue(1, 0, xie.y);
    input->SetValue(2, 0, xie.z);

    input->SetValue(0, 1, xiep.x);
    input->SetValue(1, 1, xiep.y);
    input->SetValue(2, 1, xiep.z);

    input->SetValue(0, 2, xid.x);
    input->SetValue(1, 2, xid.y);
    input->SetValue(2, 2, xid.z);

    input->SetValue(0, 4, xidpp.x);
    input->SetValue(1, 4, xidpp.y);
    input->SetValue(2, 4, xidpp.z);

    input->SetValue(0, 5, xidppp.x);
    input->SetValue(1, 5, xidppp.y);
    input->SetValue(2, 5, xidppp.z);

    input->SetValue(0, 6, w.x);
    input->SetValue(1, 6, w.y);
    input->SetValue(2, 6, w.z);

    input->SetValue(0, 7, q.q0);
    input->SetValue(1, 7, q.q1);
    input->SetValue(2, 7, q.q2);
    input->SetValue(3, 7, q.q3);

    input->SetValue(0, 8, F.x);
    input->SetValue(1, 8, F.y);
    input->SetValue(2, 8, F.z);

    input->SetValue(0, 9, Fd.x);
    input->SetValue(1, 9, Fd.y);
    input->SetValue(2, 9, Fd.z);
}

void Sliding_force::UseDefaultPlot(const LayoutPosition *position) {
    DataPlot1D *rollg = new DataPlot1D(position, "u_roll", -1, 1);
    rollg->AddCurve(state->Element(0));
    
}

void Sliding_force::UseDefaultPlot2(const LayoutPosition *position) {
    DataPlot1D *pitchg = new DataPlot1D(position, "u_pitch", -1, 1);
    pitchg->AddCurve(state->Element(1));
    
}

void Sliding_force::UseDefaultPlot3(const LayoutPosition *position) {
    DataPlot1D *yawg = new DataPlot1D(position, "u_yaw", -1, 1);
    yawg->AddCurve(state->Element(2));
    
}

void Sliding_force::UseDefaultPlot4(const LayoutPosition *position) {    
    DataPlot1D *uz = new DataPlot1D(position, "u_z", -1, 1);
    uz->AddCurve(state->Element(3));
    
}

void Sliding_force::UseDefaultPlot5(const LayoutPosition *position) {    
    DataPlot1D *r = new DataPlot1D(position, "r", -3.14, 3.14);
    r->AddCurve(state->Element(26));
    r->AddCurve(state->Element(4), DataPlot::Blue);
    
}

void Sliding_force::UseDefaultPlot6(const LayoutPosition *position) {    
    DataPlot1D *p = new DataPlot1D(position, "p", -3.14, 3.14);
    p->AddCurve(state->Element(27));
    p->AddCurve(state->Element(5), DataPlot::Blue);
    //p->AddCurve(state->Element(25), DataPlot::Blue);
    
}

void Sliding_force::UseDefaultPlot7(const LayoutPosition *position) {    
    DataPlot1D *y = new DataPlot1D(position, "y", -3.14, 3.14);
    y->AddCurve(state->Element(28));
    y->AddCurve(state->Element(6)),DataPlot::Blue;
    
}

void Sliding_force::UseDefaultPlot8(const LayoutPosition *position) {    
    DataPlot1D *Sp = new DataPlot1D(position, "S_p", -5, 5);
    Sp->AddCurve(state->Element(7), DataPlot::Red);
    Sp->AddCurve(state->Element(8), DataPlot::Green);
    Sp->AddCurve(state->Element(9), DataPlot::Blue);
    
}

void Sliding_force::UseDefaultPlot9(const LayoutPosition *position) {    
    DataPlot1D *Sq = new DataPlot1D(position, "S_a", -5, 5);
    Sq->AddCurve(state->Element(10), DataPlot::Green);
    Sq->AddCurve(state->Element(11), DataPlot::Red);
    Sq->AddCurve(state->Element(12), DataPlot::Black);
    
}

void Sliding_force::UseDefaultPlot10(const LayoutPosition *position) {    
    DataPlot1D *Sf = new DataPlot1D(position, "Svf", -5, 5);
    //Sf->AddCurve(state->Element(13), DataPlot::Green);
    Sf->AddCurve(state->Element(14), DataPlot::Red);
    Sf->AddCurve(state->Element(15), DataPlot::Black);
    
}

void Sliding_force::UseDefaultPlot11(const LayoutPosition *position) {    
    DataPlot1D *Sr = new DataPlot1D(position, "Sr", -5, 5);
    Sr->AddCurve(state->Element(16), DataPlot::Green);
    Sr->AddCurve(state->Element(17), DataPlot::Red);
    Sr->AddCurve(state->Element(18), DataPlot::Black);
    
}

void Sliding_force::UseDefaultPlot12(const LayoutPosition *position) {    
    DataPlot1D *Fx = new DataPlot1D(position, "Fx", -5, 5);
    Fx->AddCurve(state->Element(19), DataPlot::Black);
    Fx->AddCurve(state->Element(22), DataPlot::Blue);
}

void Sliding_force::UseDefaultPlot13(const LayoutPosition *position) {    
    DataPlot1D *Fy = new DataPlot1D(position, "Fy", -5, 5);
    Fy->AddCurve(state->Element(20), DataPlot::Black);
    Fy->AddCurve(state->Element(23), DataPlot::Blue);
}

void Sliding_force::UseDefaultPlot14(const LayoutPosition *position) {    
    DataPlot1D *Fz = new DataPlot1D(position, "Fz", -5, 5);
    Fz->AddCurve(state->Element(21), DataPlot::Black);
    Fz->AddCurve(state->Element(24), DataPlot::Blue);
}

void Sliding_force::UseDefaultPlot15(const LayoutPosition *position) {    
    DataPlot1D *tau_roll = new DataPlot1D(position, "tau_roll", -5, 5);
    tau_roll->AddCurve(state->Element(29), DataPlot::Black);
}

void Sliding_force::UseDefaultPlot16(const LayoutPosition *position) {    
    DataPlot1D *tau_pitch = new DataPlot1D(position, "tau_pitch", -5, 5);
    tau_pitch->AddCurve(state->Element(30), DataPlot::Black);
}

void Sliding_force::UseDefaultPlot17(const LayoutPosition *position) {    
    DataPlot1D *tau_yaw = new DataPlot1D(position, "tau_yaw", -5, 5);
    tau_yaw->AddCurve(state->Element(31), DataPlot::Black);
}

void Sliding_force::UseDefaultPlot18(const LayoutPosition *position) {    
    DataPlot1D *u_x = new DataPlot1D(position, "u_x", -5, 5);
    u_x->AddCurve(state->Element(32), DataPlot::Black);
}

void Sliding_force::UseDefaultPlot19(const LayoutPosition *position) {    
    DataPlot1D *u_y = new DataPlot1D(position, "u_y", -5, 5);
    u_y->AddCurve(state->Element(33), DataPlot::Black);
}

void Sliding_force::UseDefaultPlot20(const LayoutPosition *position) {    
    DataPlot1D *u_z = new DataPlot1D(position, "u_z", -5, 5);
    u_z->AddCurve(state->Element(34), DataPlot::Black);
}

void Sliding_force::UseDefaultPlot21(const LayoutPosition *position) {    
    DataPlot1D *T = new DataPlot1D(position, "T", -10, 10);
    T->AddCurve(state->Element(35), DataPlot::Black);
}

void Sliding_force::UseDefaultPlot22(const LayoutPosition *position) {    
    DataPlot1D *Levant_err = new DataPlot1D(position, "Levant_err", -1, 1);
    Levant_err->AddCurve(state->Element(36), DataPlot::Black);
    Levant_err->AddCurve(state->Element(37), DataPlot::Blue);
    Levant_err->AddCurve(state->Element(38), DataPlot::Red);
}



void Sliding_force::UpdateFrom(const io_data *data) {
    tactual = double(GetTime())/1000000000-t0;
    //Printf("tactual: %f\n",tactual);
    float Trs=0, tau_roll=0, tau_pitch=0, tau_yaw=0, Tr=0;

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

    Eigen::Vector3f xie(input->ValueNoMutex(0, 0),input->ValueNoMutex(1, 0),input->ValueNoMutex(2, 0));
    Eigen::Vector3f xiep(input->ValueNoMutex(0, 1),input->ValueNoMutex(1, 1),input->ValueNoMutex(2, 1));

    Eigen::Vector3f xid(input->ValueNoMutex(0, 2),input->ValueNoMutex(1, 2),input->ValueNoMutex(2, 2));
    Eigen::Vector3f xidpp(input->ValueNoMutex(0, 4),input->ValueNoMutex(1, 4),input->ValueNoMutex(2, 4));
    Eigen::Vector3f xidppp(input->ValueNoMutex(0, 5),input->ValueNoMutex(1, 5),input->ValueNoMutex(2, 5));

    Eigen::Vector3f w(input->ValueNoMutex(0, 6),input->ValueNoMutex(1, 6),input->ValueNoMutex(2, 6));

    Eigen::Quaternionf q(input->ValueNoMutex(0, 7),input->ValueNoMutex(1, 7),input->ValueNoMutex(2, 7),input->ValueNoMutex(3, 7));

    Eigen::Vector3f F(-input->ValueNoMutex(0, 8),input->ValueNoMutex(1, 8),input->ValueNoMutex(2, 8));
    Eigen::Vector3f Fd(input->ValueNoMutex(0, 9),input->ValueNoMutex(1, 9),input->ValueNoMutex(2, 9));
    
    input->ReleaseMutex();

    Eigen::Vector3f u_p;
    Eigen::Vector3f u;

    Eigen::Vector3f u_f;

    Eigen::Quaternionf qd;
    //Eigen::Vector3f wd;

    // std::cout << "F: " << F << std::endl;
    // std::cout << "Fd: " << Fd << std::endl;

    Position(u_p,xie,xiep,xid,xidpp,xidppp,q);
    //Printf("Pos\n");
    ForcePosition(u_f,xie,xiep,xid,xidpp,xidppp,q,F,Fd);
    //Printf("Force-Pos\n");

    float psi = tanhf(mu_t->Value()*F.norm());

    std::cout << "Psi: " << psi << std::endl;

    u = 0.5*psi*u_f + (1-0.5*psi)*u_p;

    //Printf("U\n");

    Trs = u.norm();

    Eigen::Vector3f alpha_l(alpha_l1->Value(),alpha_l2->Value(),alpha_l3->Value());
    Eigen::Vector3f lambda_l(lamb_l1->Value(),lamb_l2->Value(),lamb_l3->Value());

    levant.setParam_vec(alpha_l, lambda_l);
    Eigen::Vector3f up = levant.Compute(u,delta_t);

    Eigen::Vector3f err = levant.getErr_v();

    // Eigen::Vector3f un = u.normalized();
    // Eigen::Vector3f upn = ((u.transpose()*u)*up - (u.transpose()*up)*u)/(powf(u.norm(),3));


    Eigen::Vector3f uh = u.normalized();
    Eigen::Vector3f uph = ((u.transpose()*u)*up - (u.transpose()*up)*u)/(powf(u.norm(),3));

    //Printf("Norm\n");
    qd = Eigen::Quaternionf( (0.5)*sqrtf(-2*uh(2)+2) , uh(1)/sqrtf(-2*uh(2)+2), -uh(0)/sqrtf(-2*uh(2)+2), 0);

    Eigen::Quaternionf qdp(-(0.5)*(uph(2)/sqrtf(-2*uh(2)+2)),
                            (uph(1)/sqrtf(-2*uh(2)+2)) + ((uh(1)*uph(2))/powf(-2*uh(2)+2,1.5)),
                            -(uph(0)/sqrtf(-2*uh(2)+2)) - ((uh(0)*uph(2))/powf(-2*uh(2)+2,1.5)),
                            0);
    
    Eigen::Vector3f wd = 2*(qd.conjugate()*qdp).vec();

    //wd << (uph(1) - ( (uh(1)*uph(2))/(1-uh(2)) ),  -uph(0) + ( (uh(0)*uph(2))/(1-uh(2)) ), (uh(1)*uph(0) - uh(0)*uph(1))/(1-uh(2)));
    //Printf("Diff\n");


    
    Quaternion qd2 = Quaternion(qd.w(),qd.x(),qd.y(),qd.z());
    Euler etad = qd2.ToEuler();

    Quaternion q2 = Quaternion(q.w(),q.x(),q.y(),q.z());
    Euler eta = q2.ToEuler();

    //Eigen::Vector3f eta = q.toRotationMatrix().eulerAngles(0, 1, 2);

    Eigen::Vector3f tau;

    Orientation(tau,qd,wd,q,w);
    //Printf("Tau\n");

    // Eigen::Matrix3f Wi;

    // Wi << 1, sinf(eta.roll)*tanf(eta.pitch), tanf(eta.pitch)*cosf(eta.roll),
    //         0, cosf(eta.roll), -sinf(eta.roll),
    //         0, sinf(eta.roll)/cosf(eta.pitch), cosf(eta.roll)/cosf(eta.pitch);

    // Eigen::Vector3f etap = Wi*w;

    // float pitchd = Fd(0)/(m->Value()*g->Value()); //F(0)
    // float pitchdp = 0;

    //ForcePitch(tau(1),F(2)-Fd(2),eta.pitch-pitchd,etap(1)-pitchdp); //F(0)-Fd(0)
    
    tau_roll = (float)tau(0)/km->Value();
    
    tau_pitch = (float)tau(1)/km->Value();
    
    tau_yaw = (float)tau(2)/km->Value();
    
    Tr = Trs/km_z->Value();
    
    tau_roll = -Sat(tau_roll,sat_r->Value());
    tau_pitch = -Sat(tau_pitch,sat_p->Value());
    tau_yaw = -Sat(tau_yaw,sat_y->Value());
    Tr = -Sat(Tr,sat_t->Value());
    
    state->GetMutex();
    state->SetValueNoMutex(0, 0, tau_roll);
    state->SetValueNoMutex(1, 0, tau_pitch);
    state->SetValueNoMutex(2, 0, tau_yaw);
    state->SetValueNoMutex(3, 0, Tr);
    state->SetValueNoMutex(4, 0, etad.roll);
    state->SetValueNoMutex(5, 0, etad.pitch);
    state->SetValueNoMutex(6, 0, etad.yaw);
    state->SetValueNoMutex(19, 0, F(0));
    state->SetValueNoMutex(20, 0, F(1));
    state->SetValueNoMutex(21, 0, F(2));
    state->SetValueNoMutex(22, 0, Fd(0));
    state->SetValueNoMutex(23, 0, Fd(1));
    state->SetValueNoMutex(24, 0, Fd(2));
    //state->SetValueNoMutex(25, 0, pitchd);
    state->SetValueNoMutex(35, 0, Trs);
    state->SetValueNoMutex(36, 0, err(0));
    state->SetValueNoMutex(37, 0, err(1));
    state->SetValueNoMutex(38, 0, err(2));
    state->SetValueNoMutex(26, 0, eta.roll);
    state->SetValueNoMutex(27, 0, eta.pitch);
    state->SetValueNoMutex(28, 0, eta.yaw);
    state->SetValueNoMutex(29, 0, tau(0));
    state->SetValueNoMutex(30, 0, tau(1));
    state->SetValueNoMutex(31, 0, tau(2));
    state->ReleaseMutex();


    output->SetValue(0, 0, tau_roll);
    output->SetValue(1, 0, tau_pitch);
    output->SetValue(2, 0, tau_yaw);
    output->SetValue(3, 0, Tr);
    output->SetDataTime(data->DataTime());
    
    ProcessUpdate(output);
    
}

// void Sliding_force::ForcePitch(float &tau_pitch, const float dFx, const float dPitch, const float dPitchp){
//     Printf("dFx: %f\n",dFx);
//     Printf("dPitchp: %f\n",dPitchp);
//     float Sfx = dPitchp + (1/(2*m->Value()*g->Value()))*alpha_fx->Value()*dFx;

//     sgnfxp = signth(Sfx,mu->Value());

//     sgnfx = rk4(function1d,sgnfx,sgnfxp,delta_t);

//     tau_pitch = -kf_x->Value()*(Sfx + gamma_fx->Value()*sgnfx);

//     Printf("tau_pitch: %f\n",tau_pitch);

//     state->GetMutex();
//     state->SetValueNoMutex(13, 0, Sfx);
//     state->ReleaseMutex();

// }

void Sliding_force::ForcePosition(Eigen::Vector3f &u, const Eigen::Vector3f xie, const Eigen::Vector3f xiep, const Eigen::Vector3f xid,
                            const Eigen::Vector3f xidpp, const Eigen::Vector3f xidppp, const Eigen::Quaternionf q, const Eigen::Vector3f F, const Eigen::Vector3f Fd){

    // Eigen::Quaternionf qd;
    // Eigen::Vector3f wd;
    // float Trs;

    Eigen::Vector2f gammaf_v(gamma_fy->Value(), gamma_fz->Value());
    Eigen::Matrix2f gammaf = gammaf_v.asDiagonal();

    Eigen::Vector2f beta_v(beta_1->Value(), beta_2->Value());
    Eigen::Matrix2f beta = beta_v.asDiagonal();

    Eigen::Vector2f eta_v(eta_1->Value(), eta_2->Value());
    Eigen::Matrix2f eta = eta_v.asDiagonal();

    Eigen::Vector3f alphap_v(alpha_x->Value(), alpha_y->Value(), alpha_z->Value());
    Eigen::Matrix3f alphap = alphap_v.asDiagonal();

    float alphapf = alphapf1->Value();

    Eigen::Vector3f gammap_v(gamma_x->Value(), gamma_y->Value(), gamma_z->Value());
    Eigen::Matrix3f gammap = gammap_v.asDiagonal();

    Eigen::Vector3f Kpv(Kp_x->Value(), Kp_y->Value(), Kp_z->Value());
    Eigen::Matrix3f Kpm = Kpv.asDiagonal();

    Eigen::Vector3f ez(0,0,1);
    Eigen::Matrix<float, 2, 3> Jphi_xi;
    Eigen::Matrix<float, 3, 2> Jphi_xi_pinv;

    Jphi_xi << 0, 1, 0,
               0, 0, 1;
    
    //Jphi_xi_pinv = Jphi_xi.transpose()*(Jphi_xi*Jphi_xi.transpose()).inverse();
    Jphi_xi_pinv << 0, 0,
                    1, 0,
                    0, 1;

    Eigen::Matrix3f Q = I - Jphi_xi_pinv*Jphi_xi;

    // Force control

    Eigen::Vector2f lambd(Fd(1), Fd(2));

    float dF = F(0)-Fd(0);

    dFi_ = rk4(function1d,dFi_,dF,delta_t);

    Eigen::Vector3f dFi(dFi_,0,0);

    //std::cout << "dFi: \n" << dFi << std::endl;

    dLamb(0) = F(1)-Fd(1);
    dLamb(1) = F(2)-Fd(2);

    //std::cout << "dLamb: \n" << dLamb << std::endl;

    Sf(0) = rk4(function1d,Sf(0),dLamb(0),delta_t);
    Sf(1) = rk4(function1d,Sf(1),dLamb(1),delta_t);

    //Sf = dLamb;

    //std::cout << "Sf: \n" << Sf << std::endl;

    Eigen::Vector2f Sdf0 = 0.0*Eigen::Vector2f(1,1);

    Eigen::Vector2f Sdf = Sdf0*exp(-kf->Value()*(tactual));

    Eigen::Vector2f Sdfp = -kf->Value()*Sdf;

    Eigen::Vector2f Sqf = Sf - Sdf;

    sgnfp(0) = signth(Sqf(0),muf->Value());
    sgnfp(1) = signth(Sqf(1),muf->Value());


    sgnf(0) = rk4(function1d,sgnf(0),sgnfp(0),delta_t);
    sgnf(1) = rk4(function1d,sgnf(1),sgnfp(1),delta_t);

    Eigen::Vector2f Svf = Sqf + gammaf*sgnf;

    // Position control
    flair::core::Time t0_p = GetTime();

    Eigen::Vector3f nup = xiep + alphap*xie - alphapf*dFi;

    sgnpos_p = signth(nup,mup->Value());
    sgnpos = rk4_vec(sgnpos, sgnpos_p, delta_t);

    Eigen::Vector3f nurp = nup + gammap*sgnpos;

    //Eigen::Vector3f xirpp = xidpp - alphap*xiep - gammap*sgnpos_p + alphapf*Eigen::Vector3f(dF,0,0);

    Eigen::Vector3f Sr = Q*nurp - Jphi_xi_pinv*beta*Svf;

    u = -Kpm*Sr + Jphi_xi_pinv*(-lambd + Sdfp + gammaf*sgnfp + eta*Svf) - m->Value()*g->Value()*ez; //+ m->Value()*xirpp

    //Trs = u.norm();

    // levant.setParam(alpha_l->Value(), lamb_l->Value());
    // Eigen::Vector3f up = levant.Compute(u,delta_t);

    // Eigen::Vector3f err = levant.getErr_v();

    // Eigen::Vector3f un = u.normalized();
    // Eigen::Vector3f upn = ((u.transpose()*u)*up - (u.transpose()*up)*u)/(powf(u.norm(),3));


    //qd = Eigen::Quaternionf( (0.5)*sqrtf(-2*un(2)+2) , un(1)/sqrtf(-2*un(2)+2), -un(0)/sqrtf(-2*un(2)+2), 0);

    // Eigen::Quaternionf qdp(-(0.5)*(upn(2)/sqrtf(-2*un(2)+2)),
    //                         (upn(1)/sqrtf(-2*un(2)+2)) + ((un(1)*upn(2))/powf(-2*un(2)+2,1.5)),
    //                         -(upn(0)/sqrtf(-2*un(2)+2)) - ((un(0)*upn(2))/powf(-2*un(2)+2,1.5)),
    //                         0);
    
    //Eigen::Vector3f wd = 2*(qd.conjugate()*qdp).vec();

    //wd << upn(1) - ( (un(1)*upn(2))/(1-un(2)) ), -upn(0) + ( (un(0)*upn(2))/(1-un(2)) ),  (un(1)*upn(0) - un(0)*upn(1))/(1-un(2));

    flair::core::Time dt_pos = GetTime() - t0_p;

    //lp->SetText("Latecia pos: %.3f ms",(float)dt_pos/1000000);

    state->GetMutex();
    state->SetValueNoMutex(7, 0, nup.x());
    state->SetValueNoMutex(8, 0, nup.y());
    state->SetValueNoMutex(9, 0, nup.z());
    state->SetValueNoMutex(15, 0, Sqf(1));
    state->SetValueNoMutex(14, 0, Sqf(0));
    state->SetValueNoMutex(16, 0, Sr(0));
    state->SetValueNoMutex(17, 0, Sr(1));
    state->SetValueNoMutex(18, 0, Sr(2));
    state->SetValueNoMutex(32, 0, u(0));
    state->SetValueNoMutex(33, 0, u(1));
    state->SetValueNoMutex(34, 0, u(2));
    //state->SetValueNoMutex(35, 0, Trs);
    // state->SetValueNoMutex(36, 0, err(0));
    // state->SetValueNoMutex(37, 0, err(1));
    // state->SetValueNoMutex(38, 0, err(2));
    state->ReleaseMutex();

}


void Sliding_force::Position(Eigen::Vector3f &u, const Eigen::Vector3f xie, const Eigen::Vector3f xiep, const Eigen::Vector3f xid,
                            const Eigen::Vector3f xidpp, const Eigen::Vector3f xidppp, const Eigen::Quaternionf q){
    
    Eigen::Quaternionf qd;
    Eigen::Vector3f wd;
    float Trs;

    Eigen::Vector3f ez(0,0,1);
    
    //Eigen::Vector3f alphap_v(alpha_x->Value(), alpha_y->Value(), alpha_z->Value());
    Eigen::Matrix3f alphap = Eigen::Vector3f(alpha_x->Value(), alpha_y->Value(), alpha_z->Value()).asDiagonal();

    //Eigen::Vector3f gammap_v(gamma_x->Value(), gamma_y->Value(), gamma_z->Value());
    Eigen::Matrix3f gammap = Eigen::Vector3f(gamma_x->Value(), gamma_y->Value(), gamma_z->Value()).asDiagonal();

    //Eigen::Vector3f Kpv(Kp_x->Value(), Kp_y->Value(), Kp_z->Value());
    Eigen::Matrix3f Kpm = Eigen::Vector3f(Kp_x->Value(), Kp_y->Value(), Kp_z->Value()).asDiagonal();

    flair::core::Time t0_p = GetTime();

    Eigen::Vector3f nup = xiep + alphap*xie;

    sgnpos_p = signth(nup,1);
    sgnpos = rk4_vec(sgnpos, sgnpos_p, delta_t);

    Eigen::Vector3f nurp = nup + gammap*sgnpos;

    Eigen::Vector3f xirpp = xidpp - alphap*xiep - gammap*sgnpos_p;


    u = -Kpm*nurp - m->Value()*g->Value()*ez + m->Value()*xirpp; //- m->Value()*g->Value()*ez + m->Value()*xirpp

    //Trs = u.norm();

    //Eigen::Vector3f Qe3 = q.toRotationMatrix()*ez;
    
    //Eigen::Vector3f Lambpv(powf(sech(nup(0)*1),2), powf(sech(nup(1)*1),2), powf(sech(nup(2)*1),2) );
    //Eigen::Matrix3f Lambp = Lambpv.asDiagonal();

    //Eigen::Vector3f vec(sin(tactual), sin(tactual), sin(tactual));

    // float f = gammap->Value()*sin(alphap->Value()*tactual);
    // float alpha2 = Kp->Value();
    // float lamb = Kd->Value();

    

    //ud = levant.Compute(f,delta_t);

    //Eigen::Vector3f up;
    //Eigen::Vector3f ud;
    // if(levantd->IsChecked()){
    //     levant.setParam(alpha_l->Value(), lamb_l->Value());
    //     up = levant.Compute(vec,delta_t);
    // }else{
    //     up = -(Kpm + m->Value()*alphap + m->Value()*gammap*Lambp) * (g->Value()*ez - (Trs/m->Value())*Qe3 - xidpp) 
    //                     -alphap*(Kpm+m->Value()*gammap*Lambp)*xiep - Kpm*gammap*sgnpos_p + m->Value()*xidppp;
    // }

    
    flair::core::Time dt_pos = GetTime() - t0_p;

    //lp->SetText("Latecia pos: %.3f ms",(float)dt_pos/1000000);

    state->GetMutex();
    state->SetValueNoMutex(7, 0, nup.x());
    state->SetValueNoMutex(8, 0, nup.y());
    state->SetValueNoMutex(9, 0, nup.z());
    state->ReleaseMutex();
    
} 

void Sliding_force::Orientation(Eigen::Vector3f &tau, const Eigen::Quaternionf qd, const Eigen::Vector3f wd, const Eigen::Quaternionf q, const Eigen::Vector3f w){
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

    tau = -Kdm*nur;

    flair::core::Time dt_ori = GetTime() - t0_o;

    //lo->SetText("Latecia ori: %.3f ms",(float)dt_ori/1000000);

    state->GetMutex();
    state->SetValueNoMutex(10, 0, nuq.x());
    state->SetValueNoMutex(11, 0, nuq.y());
    state->SetValueNoMutex(12, 0, nuq.z());
    state->ReleaseMutex();

}

float Sliding_force::Sat(float value, float borne) {
    if (value < -borne)
        return -borne;
    if (value > borne)
        return borne;
    return value;
}

float Sliding_force::sech(float value) {
    return 1 / coshf(value);
}

} // end namespace filter
} // end namespace flair
