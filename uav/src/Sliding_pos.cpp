// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2023/01/01
//  filename:   Sliding_pos.cpp
//
//  author:     Sergio Urzua
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Class defining a position sliding mode controller
//
//
/*********************************************************************/
#include "Sliding_pos.h"
#include "NMethods.h"
//#include "AC1.h"
#include <Eigen/src/Core/Matrix.h>
#include <Matrix.h>
#include <Object.h>
#include <Thread.h>
#include <Vector3D.h>
#include <TabWidget.h>
#include <CheckBox.h>
#include <Quaternion.h>
#include <Eigen/Geometry>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <algorithm>
#include <cmath>
#include <Euler.h>
#include <Label.h>
//#include <iostream>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

namespace flair {
namespace filter {

Sliding_pos::Sliding_pos(const LayoutPosition *position, string name): ControlLaw(position->getLayout(), name, 4){ // Salidas 4
    first_update = true;
    // init matrix
    input = new Matrix(this, 4, 8, floatType, name);

    MatrixDescriptor *desc = new MatrixDescriptor(26, 1);
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
    desc->SetElementName(13, 0, "qe_0");
    desc->SetElementName(14, 0, "qe_1");
    desc->SetElementName(15, 0, "qe_2");
    desc->SetElementName(16, 0, "qe_3");
    desc->SetElementName(17, 0, "xie_x");
    desc->SetElementName(18, 0, "xie_y");
    desc->SetElementName(19, 0, "xie_z");
    desc->SetElementName(20, 0, "u_x");
    desc->SetElementName(21, 0, "u_y");
    desc->SetElementName(22, 0, "u_z");
    desc->SetElementName(23, 0, "tau_roll");
    desc->SetElementName(24, 0, "tau_pitch");
    desc->SetElementName(25, 0, "tau_yaw");

    state = new Matrix(this, desc, floatType, name);
    delete desc;


    GroupBox *reglages_groupbox = new GroupBox(position, name);
    GroupBox *num = new GroupBox(reglages_groupbox->NewRow(), "Integral y derivada");
    GroupBox *ori = new GroupBox(reglages_groupbox->NewRow(), "Orientacion");
    GroupBox *pos = new GroupBox(reglages_groupbox->NewRow(), "Posicion");
    GroupBox *mot = new GroupBox(reglages_groupbox->NewRow(), "Motores");
    GroupBox *ac11 = new GroupBox(reglages_groupbox->NewRow(), "AC1");
    GroupBox *ac12 = new GroupBox(reglages_groupbox->NewRow(), "AC2");

    T = new DoubleSpinBox(num->NewRow(), "period, 0 for auto", " s", 0, 1, 0.01,3);
    alpha_l = new DoubleSpinBox(num->NewRow(), "alpha Levant:", 0, 5000, 0.001, 3);
    lamb_l = new DoubleSpinBox(num->LastRowLastCol(), "lambda Levant:", 0, 5000, 0.001, 3);
    levantd = new CheckBox(num->LastRowLastCol(), "Levant");

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
    
    

    sat_r = new DoubleSpinBox(mot->NewRow(), "sat roll:", 0, 1, 0.1);
    sat_p = new DoubleSpinBox(mot->LastRowLastCol(), "sat pitch:", 0, 1, 0.1);
    sat_y = new DoubleSpinBox(mot->LastRowLastCol(), "sat yaw:", 0, 1, 0.1);
    sat_t = new DoubleSpinBox(mot->LastRowLastCol(), "sat thrust:", 0, 1, 0.1);
    
    km = new DoubleSpinBox(mot->NewRow(), "km:", -100, 100, 0.01, 3);
    km_z = new DoubleSpinBox(mot->LastRowLastCol(), "km_z:", -100, 100, 0.01, 3);
    
    m = new DoubleSpinBox(pos->NewRow(),"m",0,2000,0.001,3);
    g = new DoubleSpinBox(pos->LastRowLastCol(),"g",-10,10,0.01,3);
    p_1 = new DoubleSpinBox(pos->LastRowLastCol(), "p:", 0, 50000, 1, 3);
    //lp = new Label(pos->LastRowLastCol(), "Latencia pos");
    
    t0 = double(GetTime())/1000000000;

    ac1 = new AC1(ac11, "AC_ori_full");
    ac2 = new AC1(ac12, "AC_pos_full");


    levant = Levant_diff("tanh", 8, 6, 3000);
    levant3 = Levant3(1, 8, 3000.0);

    sgnpos_p << 0,0,0;
    sgnpos << 0,0,0;

    sgnori_p << 0,0,0;
    sgnori << 0,0,0;
    
    AddDataToLog(state);
    AddDeviceToLog(ac1);
    AddDeviceToLog(ac2);
}

Sliding_pos::~Sliding_pos(void) {
     
    delete input;
    delete state;
    delete ac1;
    delete ac2;
    
}

void Sliding_pos::Reset(void) {
    first_update = true;
    //t0 = 0;
    t0 = double(GetTime())/1000000000;
    sgnori_p = Eigen::Vector3f::Zero();
    sgnori = Eigen::Vector3f::Zero();
    sgnpos_p = Eigen::Vector3f::Zero();
    sgnpos = Eigen::Vector3f::Zero();

    levant.Reset();
    levant3.Reset();
    ac1->Reset();
    ac2->Reset();

    dum = 0.0F;

    // state->GetMutex();
    // for (int i = 0; i < 26; ++i) {
    //     state->SetValueNoMutex(i, 0, 0.0F);
    // }
    // state->ReleaseMutex();

    // output->SetValue(0, 0, 0.0F);
    // output->SetValue(1, 0, 0.0F);
    // output->SetValue(2, 0, 0.0F);
    // output->SetValue(3, 0, 0.0F);

    

    // sgnpos2 = Vector3ff(0,0,0);
    // sgn2 = Vector3ff(0,0,0);

    // sgnpos << 0,0,0;
    // sgn << 0,0,0;



//    pimpl_->i = 0;
//    pimpl_->first_update = true;
}

void Sliding_pos::SetValues(Vector3Df xie, Vector3Df xiep, Vector3Df xid, Vector3Df xidpp, Vector3Df xidppp, Vector3Df w, Quaternion q){

    // float xe = xie.x;
    // float ye = xie.y;
    // float ze = xie.z;

    // float xep = xiep.x;
    // float yep = xiep.y;
    // float zep = xiep.z;

    // float xd = xid.x;
    // float yd = xid.y;
    // float zd = xid.z;

    // float xdp = xidp.x;
    // float ydp = xidp.y;
    // float zdp = xidp.z;

    // float xdpp = xidpp.x;
    // float ydpp = xidpp.y;
    // float zdpp = xidpp.z;

    // float xdppp = xidppp.x;
    // float ydppp = xidppp.y;
    // float zdppp = xidppp.z;

    // float wex = we.x;
    // float wey = we.y;
    // float wez = we.z;

    // float q0 = q.q0;
    // float q1 = q.q1;
    // float q2 = q.q2;
    // float q3 = q.q3;

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


//   input->SetValue(0, 0, ze);
//   input->SetValue(1, 0, wex);
//   input->SetValue(2, 0, wey);
//   input->SetValue(3, 0, wez);
//   input->SetValue(4, 0, zp);

//   input->SetValue(0, 1, q0);
//   input->SetValue(1, 1, q1);
//   input->SetValue(2, 1, q2);
//   input->SetValue(3, 1, q3);

//   input->SetValue(0, 2, qd0);
//   input->SetValue(1, 2, qd1);
//   input->SetValue(2, 2, qd2);
//   input->SetValue(3, 2, qd3);
}

void Sliding_pos::UseDefaultPlot(const LayoutPosition *position) {
    DataPlot1D *rollg = new DataPlot1D(position, "u_roll", -1, 1);
    rollg->AddCurve(state->Element(0));
    
}

void Sliding_pos::UseDefaultPlot2(const LayoutPosition *position) {
    DataPlot1D *pitchg = new DataPlot1D(position, "u_pitch", -1, 1);
    pitchg->AddCurve(state->Element(1));
    
}

void Sliding_pos::UseDefaultPlot3(const LayoutPosition *position) {
    DataPlot1D *yawg = new DataPlot1D(position, "u_yaw", -1, 1);
    yawg->AddCurve(state->Element(2));
    
}

void Sliding_pos::UseDefaultPlot4(const LayoutPosition *position) {    
    DataPlot1D *uz = new DataPlot1D(position, "u_z", -1, 1);
    uz->AddCurve(state->Element(3));
    
}

void Sliding_pos::UseDefaultPlot5(const LayoutPosition *position) {    
    DataPlot1D *r = new DataPlot1D(position, "r", -3.14, 3.14);
    r->AddCurve(state->Element(4));
    
}

void Sliding_pos::UseDefaultPlot6(const LayoutPosition *position) {    
    DataPlot1D *p = new DataPlot1D(position, "p", -3.14, 3.14);
    p->AddCurve(state->Element(5));
    
}

void Sliding_pos::UseDefaultPlot7(const LayoutPosition *position) {    
    DataPlot1D *y = new DataPlot1D(position, "y", -3.14, 3.14);
    y->AddCurve(state->Element(6));
    
}

void Sliding_pos::UseDefaultPlot8(const LayoutPosition *position) {    
    DataPlot1D *Sp = new DataPlot1D(position, "S_qp", -3, 3);
    Sp->AddCurve(state->Element(7), DataPlot::Red);
    Sp->AddCurve(state->Element(8), DataPlot::Green);
    Sp->AddCurve(state->Element(9), DataPlot::Blue);
    
}

void Sliding_pos::UseDefaultPlot9(const LayoutPosition *position) {    
    DataPlot1D *Sq = new DataPlot1D(position, "S_qa", -2, 2);
    Sq->AddCurve(state->Element(10), DataPlot::Green);
    Sq->AddCurve(state->Element(11), DataPlot::Red);
    Sq->AddCurve(state->Element(12), DataPlot::Black);
    
}

void Sliding_pos::UseDefaultPlot10(const LayoutPosition *position) {    
    ac1->UseDefaultPlot(position);
}

void Sliding_pos::UseDefaultPlot11(const LayoutPosition *position) {    
    ac1->UseDefaultPlot2(position);
    
}

void Sliding_pos::UseDefaultPlot12(const LayoutPosition *position) {    
    ac2->UseDefaultPlot(position);
}

void Sliding_pos::UseDefaultPlot13(const LayoutPosition *position) {    
    ac2->UseDefaultPlot2(position);
    
}

void Sliding_pos::UseDefaultPlot14(const LayoutPosition *position) {
    DataPlot1D *qe_plot = new DataPlot1D(position, "qe", -1, 1);
    qe_plot->AddCurve(state->Element(13), DataPlot::Red);
    qe_plot->AddCurve(state->Element(14), DataPlot::Green);
    qe_plot->AddCurve(state->Element(15), DataPlot::Blue);
    qe_plot->AddCurve(state->Element(16), DataPlot::Black);
}

void Sliding_pos::UseDefaultPlot15(const LayoutPosition *position) {
    DataPlot1D *xie_plot = new DataPlot1D(position, "xie", -1, 1);
    xie_plot->AddCurve(state->Element(17), DataPlot::Red);
    xie_plot->AddCurve(state->Element(18), DataPlot::Green);
    xie_plot->AddCurve(state->Element(19), DataPlot::Blue);
}

void Sliding_pos::UseDefaultPlot16(const LayoutPosition *position) {
    DataPlot1D *uc_plot = new DataPlot1D(position, "u", -5, 2);
    uc_plot->AddCurve(state->Element(20), DataPlot::Red);
    uc_plot->AddCurve(state->Element(21), DataPlot::Green);
    uc_plot->AddCurve(state->Element(22), DataPlot::Blue);
}

void Sliding_pos::UseDefaultPlot17(const LayoutPosition *position) {    
    DataPlot1D *tau_plot = new DataPlot1D(position, "tau", -1, 1);
    tau_plot->AddCurve(state->Element(23), DataPlot::Red);
    tau_plot->AddCurve(state->Element(24), DataPlot::Green);
    tau_plot->AddCurve(state->Element(25), DataPlot::Blue);
}

void Sliding_pos::UpdateFrom(const io_data *data) {
    constexpr float kEps = 1e-6f;
    
    double tactual=(double(GetTime())/1000000000)-t0;
    //Printf("tactual: %f\n",tactual);
    float Trs=0, tau_roll=0, tau_pitch=0, tau_yaw=0, Tr=0;
    const Eigen::Vector3f ez = Eigen::Vector3f::UnitZ();

    const float alpha_x_v = alpha_x->Value();
    const float alpha_y_v = alpha_y->Value();
    const float alpha_z_v = alpha_z->Value();
    const float gamma_x_v = gamma_x->Value();
    const float gamma_y_v = gamma_y->Value();
    const float gamma_z_v = gamma_z->Value();
    const float Kp_x_v = Kp_x->Value();
    const float Kp_y_v = Kp_y->Value();
    const float Kp_z_v = Kp_z->Value();
    const float alpha_roll_v = alpha_roll->Value();
    const float alpha_pitch_v = alpha_pitch->Value();
    const float alpha_yaw_v = alpha_yaw->Value();
    const float gamma_roll_v = gamma_roll->Value();
    const float gamma_pitch_v = gamma_pitch->Value();
    const float gamma_yaw_v = gamma_yaw->Value();
    const float Kd_roll_v = Kd_roll->Value();
    const float Kd_pitch_v = Kd_pitch->Value();
    const float Kd_yaw_v = Kd_yaw->Value();
    const float k_val = k->Value();
    const float p1 = p_1->Value();
    const float p_val = p->Value();
    const float g_val = g->Value();
    const float m_val = m->Value();
    const float km_val = (std::abs(km->Value()) < kEps) ? (km->Value() >= 0.0F ? kEps : -kEps) : km->Value();
    const float km_z_val = (std::abs(km_z->Value()) < kEps) ? (km_z->Value() >= 0.0F ? kEps : -kEps) : km_z->Value();

    const Eigen::Vector3f alphap_v(alpha_x_v, alpha_y_v, alpha_z_v);
    const Eigen::Vector3f gammap_v(gamma_x_v, gamma_y_v, gamma_z_v);
    const Eigen::Vector3f Kpv(Kp_x_v, Kp_y_v, Kp_z_v);
    const Eigen::Vector3f alphao_v(alpha_roll_v, alpha_pitch_v, alpha_yaw_v);
    const Eigen::Vector3f gammao_v(gamma_roll_v, gamma_pitch_v, gamma_yaw_v);
    const Eigen::Vector3f Kdv(Kd_roll_v, Kd_pitch_v, Kd_yaw_v);

    if (T->Value() == 0) {
        data->GetDataTime(now, dt1);
        //delta_t = (double)(data->DataDeltaTime()) / 1000000000.0F;
        delta_t = (double)(dt1) / 1000000000.0F;
    } else {
        data->GetDataTime(now, dt1);
        delta_t = T->Value();
    }
    Printf("delta_t: %f\n",delta_t);
    
    if (first_update) {
        delta_t = 0.0F;
        //first_update = false;
    }
    //delta_t = std::max<double>(delta_t, 0.0F);

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
    
    input->ReleaseMutex();


    if (q.norm() > kEps) {
        q.normalize();
    }

    //flair::core::Time t0_p = GetTime();

    Eigen::Vector3f nup1 = xiep + alphap_v.cwiseProduct(xie);

    if (first_update) {
        nup_t0 = nup1;
    }

    Eigen::Vector3f nupd = 0*nup_t0*exp(-k_val*(tactual));

    Eigen::Vector3f nup = nup1 - nupd;

    sgnpos_p = signth(nup,p1);
    //sgnpos_p = Eigen::Vector3f(std::cos(tactual),0,0);
    sgnpos = rk4_const(sgnpos, delta_t, sgnpos_p);
    //sgnpos = rk4_eigen(sgnpos, delta_t, sgnpos_p);
    //dum = rk4o(function1d, dum, sgnpos_p(0), delta_t);
    //dum = rk4_const(dum, delta_t, sgnpos_p(0));
    //sgnpos(2) = dum;



    Eigen::Vector3f nurp = nup + gammap_v.cwiseProduct(sgnpos);

    Eigen::Vector3f xirpp = xidpp - alphap_v.cwiseProduct(xiep) - gammap_v.cwiseProduct(sgnpos_p);

    ac2->SetValues(xie, xiep, nurp);
    ac2->Update(GetTime());
    Eigen::Vector3f NNap = Eigen::Vector3f(ac2->Output(0), ac2->Output(1), ac2->Output(2));

    //std::cout<<"NNap: " << NNap.transpose() << '\n';

    saturate(NNap, Eigen::Vector3f(-1,-1,-7), Eigen::Vector3f(1,1,0));

    Eigen::Vector3f uc = -Kpv.cwiseProduct(nurp);
    Eigen::Vector3f u = uc + NNap; //- m->Value()*g->Value()*ez + m->Value()*xirpp

    saturate(u, Eigen::Vector3f(-1,-1,-7), Eigen::Vector3f(1,1,0));

    //std::cout<<"u: " << u.transpose() << '\n';

    Trs = u.norm();

    Eigen::Vector3f Qe3 = q._transformVector(ez);

    Eigen::Vector3f Lambpv;
    const float c0 = coshf(nup(0)*p1);
    const float c1 = coshf(nup(1)*p1);
    const float c2 = coshf(nup(2)*p1);
    Lambpv(0) = p1 / (c0 * c0);
    Lambpv(1) = p1 / (c1 * c1);
    Lambpv(2) = p1 / (c2 * c2);

    //Eigen::Vector3f vec(sin(tactual), sin(tactual), sin(tactual));

    // float f = gammap->Value()*sin(alphap->Value()*tactual);
    // float alpha2 = Kp->Value();
    // float lamb = Kd->Value();

    

    //ud = levant.Compute(f,delta_t);

    Eigen::Vector3f up;
    // Eigen::Vector3f ud;
    if(levantd->IsChecked()){
        // levant.setParam(alpha_l->Value(), lamb_l->Value());
        // up = levant.Compute(u,delta_t);
        levant3.setParam(alpha_l->Value(), lamb_l->Value());
        up = levant3.compute(u,delta_t);
        //ud = levant.Compute(vec,delta_t);
    }else{
        const float safe_m = (std::abs(m_val) < kEps) ? (m_val >= 0.0F ? kEps : -kEps) : m_val;
        const Eigen::Vector3f gamma_lamb = gammap_v.cwiseProduct(Lambpv);
        const Eigen::Vector3f diag_k = Kpv + safe_m * alphap_v + safe_m * gamma_lamb;
        const Eigen::Vector3f diag_k2 = Kpv + safe_m * gamma_lamb;
        const Eigen::Vector3f term1 = -diag_k.cwiseProduct(g_val*ez - (Trs/safe_m)*Qe3 - xidpp);
        const Eigen::Vector3f term2 = -alphap_v.cwiseProduct(diag_k2.cwiseProduct(xiep));
        const Eigen::Vector3f term3 = -Kpv.cwiseProduct(gammap_v.cwiseProduct(sgnpos_p));
        const Eigen::Vector3f term4 = safe_m * xidppp;
        up = term1 + term2 + term3 + term4;
    }

    

    const float u_norm = u.norm();
    Eigen::Vector3f uh = (u_norm > kEps) ? (u / u_norm) : ez;
    Eigen::Vector3f uph = Eigen::Vector3f::Zero();
    if (u_norm > kEps) {
        const float u_norm2 = u_norm * u_norm;
        const float u_dot_up = u.dot(up);
        uph = (u_norm2 * up - u_dot_up * u) / (u_norm2 * u_norm);
    }

    //std::cout << "uph: " << uph << std::endl;


    const float denom_base = (-2.0F * uh(2)) + 2.0F;
    const float denom = std::max(denom_base, kEps);
    const float denom_sqrt = sqrtf(denom);
    const float inv_denom_sqrt = 1.0F / denom_sqrt;
    const float inv_denom_3_2 = 1.0F / (denom * denom_sqrt);

    Eigen::Quaternionf qd(0.5F * denom_sqrt, uh(1) * inv_denom_sqrt, -uh(0) * inv_denom_sqrt, 0);

    Eigen::Quaternionf qdp(-(0.5F) * (uph(2) * inv_denom_sqrt),
                            (uph(1) * inv_denom_sqrt) + ((uh(1) * uph(2)) * inv_denom_3_2),
                            -(uph(0) * inv_denom_sqrt) - ((uh(0) * uph(2)) * inv_denom_3_2),
                            0);

    Quaternion qd2 = Quaternion(qd.w(),qd.x(),qd.y(),qd.z());
    Euler eta = qd2.ToEuler();
    // Eigen::Vector3f eta = qd.toRotationMatrix().eulerAngles(0, 1, 2);

    const Eigen::Quaternionf qd_conj = qd.conjugate();
    
    
    // input = dynamic_cast<const Matrix*>(data);
  
    // if (!input) {
    //     Warn("casting %s to Matrix failed\n",data->ObjectName().c_str(),TIME_INFINITE);
    //     return;
    // }


    // input->GetMutex();

    // Eigen::Vector3f w(input->ValueNoMutex(0, 6),input->ValueNoMutex(1, 6),input->ValueNoMutex(2, 6));

    // q = Eigen::Quaternionf(input->ValueNoMutex(0, 7),input->ValueNoMutex(1, 7),input->ValueNoMutex(2, 7),input->ValueNoMutex(3, 7));
    
    // input->ReleaseMutex();

    
    // if (q.norm() > kEps) {
    //     q.normalize();
    // }
    
    
    Eigen::Quaternionf qe = q * qd_conj;

    //std::cout<<"qe: " << qe.coeffs() << std::endl;

    // Eigen::Vector3f wd(uph(1) - ( (uh(1)*uph(2))/(1-uh(2)) ), 
    //                     -uph(0) + ( (uh(0)*uph(2))/(1-uh(2)) ), 
    //                     (uh(1)*uph(0) - uh(0)*uph(1))/(1-uh(2)));

    Eigen::Vector3f wd = 2.0F*(qd_conj*qdp).vec();

    saturate(wd, Eigen::Vector3f(-1,-1,-1), Eigen::Vector3f(1,1,1));

    //std::cout<<"w: " << w << std::endl;
    //std::cout<<"wd: " << wd << std::endl;

    

    //flair::core::Time dt_pos = GetTime() - t0_p;

    //lp->SetText("Latecia pos: %.3f ms",(float)dt_pos/1000000);

    //flair::core::Time t0_o = GetTime();

    Eigen::Vector3f we = w - wd;

    //Printf("We: %f\t %f\t %f\n", we(0), we(1), we(2));

    //std::cout<<"we: " << we << std::endl;

    //Eigen::Vector3f QdTqe3 = (qd.conjugate()*qe*qd).vec();
    //Eigen::Vector3f QdTqe3 = qd.toRotationMatrix().transpose()*qe.vec();
    Eigen::Vector3f QdTqe3 = qd_conj._transformVector(qe.vec());

    //std::cout<<"QdTqe3: " << QdTqe3.coeffs() << std::endl;

    Eigen::Vector3f nu = we + alphao_v.cwiseProduct(QdTqe3);

    //std::cout<<"nu: " << nu << std::endl;
    
    //Eigen::Vector3f nu_t0 = 0.1*Eigen::Vector3f(1,1,1);

    if (first_update) {
        nu_t0 = nu;
        first_update = false;
    }
    
    Eigen::Vector3f nud = nu_t0*exp(-k_val*(tactual));
    
    Eigen::Vector3f nuq = nu-nud;

    sgnori_p = signth(nuq,p_val);
    sgnori = rk4_const(sgnori, delta_t, sgnori_p);

    Eigen::Vector3f nur = nuq + gammao_v.cwiseProduct(sgnori);

    ac1->SetValues(QdTqe3, we, nur);
    ac1->Update(GetTime());
    Eigen::Vector3f NNa = Eigen::Vector3f(ac1->Output(0), ac1->Output(1), ac1->Output(2));

    saturate(NNa, Eigen::Vector3f(-0.5,-0.5,-0.8), Eigen::Vector3f(0.5,0.5,0.8));
    
    //std::cout<<"NNa: " << NNa.transpose() << '\n';

    Eigen::Vector3f tauc = -Kdv.cwiseProduct(nur);
    Eigen::Vector3f tau = tauc + NNa; // + NNa;

    //saturate(tau, Eigen::Vector3f(-0.5,-0.5,-0.8), Eigen::Vector3f(0.5,0.5,0.8));

    //std::cout<<"tau: " << tau.transpose() << std::endl;


    //flair::core::Time dt_ori = GetTime() - t0_o;

    //lo->SetText("Latecia ori: %.3f ms",(float)dt_ori/1000000);

    
    tau_roll = (float)tau(0)/km_val;
    
    tau_pitch = (float)tau(1)/km_val;
    
    tau_yaw = (float)tau(2)/km_val;
    
    Tr = Trs/km_z_val;
    
    tau_roll = -Sat(tau_roll,sat_r->Value());
    tau_pitch = -Sat(tau_pitch,sat_p->Value());
    tau_yaw = -Sat(tau_yaw,sat_y->Value());
    Tr = -Sat(Tr,sat_t->Value());
    
    state->GetMutex();
    state->SetValueNoMutex(0, 0, tau_roll);
    state->SetValueNoMutex(1, 0, tau_pitch);
    state->SetValueNoMutex(2, 0, tau_yaw);
    state->SetValueNoMutex(3, 0, Tr);
    state->SetValueNoMutex(4, 0, eta.roll);
    state->SetValueNoMutex(5, 0, eta.pitch);
    state->SetValueNoMutex(6, 0, eta.yaw);
    state->SetValueNoMutex(7, 0, nup.x());
    state->SetValueNoMutex(8, 0, nup.y());
    state->SetValueNoMutex(9, 0, nup.z());
    state->SetValueNoMutex(10, 0, nuq.x());
    state->SetValueNoMutex(11, 0, nuq.y());
    state->SetValueNoMutex(12, 0, nuq.z());
    state->SetValueNoMutex(13, 0, qe.w());
    state->SetValueNoMutex(14, 0, qe.x());
    state->SetValueNoMutex(15, 0, qe.y());
    state->SetValueNoMutex(16, 0, qe.z());
    state->SetValueNoMutex(17, 0, xie.x());
    state->SetValueNoMutex(18, 0, xie.y());
    state->SetValueNoMutex(19, 0, xie.z());
    state->SetValueNoMutex(20, 0, uc.x());
    state->SetValueNoMutex(21, 0, uc.y());
    state->SetValueNoMutex(22, 0, uc.z());
    state->SetValueNoMutex(23, 0, tauc.x());
    state->SetValueNoMutex(24, 0, tauc.y());
    state->SetValueNoMutex(25, 0, tauc.z());
    state->ReleaseMutex();


    output->SetValue(0, 0, tau_roll);
    output->SetValue(1, 0, tau_pitch);
    output->SetValue(2, 0, tau_yaw);
    output->SetValue(3, 0, Tr);
    output->SetDataTime(data->DataTime());
    
    ProcessUpdate(output);
    
}

float Sliding_pos::Sat(float value, float borne) {
    if (value < -borne)
        return -borne;
    if (value > borne)
        return borne;
    return value;
}

float Sliding_pos::sech(float value) {
    return 1 / coshf(value);
}

} // end namespace filter
} // end namespace flair
