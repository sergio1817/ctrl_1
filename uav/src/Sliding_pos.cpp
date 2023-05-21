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
#include "Sliding_pos.h"
#include "NMethods.h"
#include <Matrix.h>
#include <Vector3D.h>
#include <Eigen/Dense>
#include <Quaternion.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <cmath>
#include <Euler.h>
#include <iostream>

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

    MatrixDescriptor *desc = new MatrixDescriptor(13, 1);
    desc->SetElementName(0, 0, "u_roll");
    desc->SetElementName(1, 0, "u_pitch");
    desc->SetElementName(2, 0, "u_yaw");
    desc->SetElementName(3, 0, "u_z");
    desc->SetElementName(4, 0, "roll");
    desc->SetElementName(5, 0, "pitch");
    desc->SetElementName(6, 0, "yaw");
    desc->SetElementName(7, 0, "nurp_x");
    desc->SetElementName(8, 0, "nurp_y");
    desc->SetElementName(9, 0, "nurp_z");
    desc->SetElementName(10, 0, "nur_roll");
    desc->SetElementName(11, 0, "nur_pitch");
    desc->SetElementName(12, 0, "nur_yaw");
    state = new Matrix(this, desc, floatType, name);
    delete desc;


    GroupBox *reglages_groupbox = new GroupBox(position, name);
    T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s", 0, 1, 0.001,6);
    GroupBox *ori = new GroupBox(reglages_groupbox->NewRow(), "Orientacion");
    GroupBox *pos = new GroupBox(reglages_groupbox->NewRow(), "Posicion");
    GroupBox *mot = new GroupBox(reglages_groupbox->NewRow(), "Motores");

    gamma_roll = new DoubleSpinBox(ori->NewRow(), "gamma_roll:", -500, 500, 0.0001, 12);
    gamma_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "gamma_pitch:", -500, 500, 0.0001, 12);
    gamma_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "gamma_yaw:", -500, 500, 0.0001, 12);
    alpha_roll = new DoubleSpinBox(ori->NewRow(), "alpha_roll:", 0, 50000, 0.5, 3);
    alpha_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "alpha_pitch:", 0, 50000, 0.5, 3);
    alpha_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "alpha_yaw:", 0, 50000, 0.5, 3);
    k = new DoubleSpinBox(ori->NewRow(), "k:", 0, 50000, 0.5, 3);
    p = new DoubleSpinBox(ori->LastRowLastCol(), "p:", 0, 50000, 1, 3);
    Kd_roll = new DoubleSpinBox(ori->NewRow(), "Kd_rol:", -5000, 50000, 0.5, 3);
    Kd_pitch = new DoubleSpinBox(ori->LastRowLastCol(), "Kd_pitch:", -5000, 50000, 0.5, 3);
    Kd_yaw = new DoubleSpinBox(ori->LastRowLastCol(), "Kd_yaw:", -5000, 50000, 0.5, 3);
    
    
   
    gamma_x = new DoubleSpinBox(pos->NewRow(), "gamma_x:", -500, 500, 0.0001, 12);
    gamma_y = new DoubleSpinBox(pos->LastRowLastCol(), "gamma_y:", -500, 500, 0.0001, 12);
    gamma_z = new DoubleSpinBox(pos->LastRowLastCol(), "gamma_z:", -500, 500, 0.0001, 12);
    alpha_x = new DoubleSpinBox(pos->NewRow(), "alpha_x:", 0, 50000, 0.5, 3);
    alpha_y = new DoubleSpinBox(pos->LastRowLastCol(), "alpha_y:", 0, 50000, 0.5, 3);
    alpha_z = new DoubleSpinBox(pos->LastRowLastCol(), "alpha_z:", 0, 50000, 0.5, 3);
    Kp_x = new DoubleSpinBox(pos->NewRow(), "Kp_x:", -5000, 50000, 0.5, 3);
    Kp_y = new DoubleSpinBox(pos->LastRowLastCol(), "Kp_y:", -5000, 50000, 0.5, 3);
    Kp_z = new DoubleSpinBox(pos->LastRowLastCol(), "Kp_z:", -5000, 50000, 0.5, 3);

    sat_r = new DoubleSpinBox(mot->NewRow(), "sat roll:", 0, 1, 0.1);
    sat_p = new DoubleSpinBox(mot->LastRowLastCol(), "sat pitch:", 0, 1, 0.1);
    sat_y = new DoubleSpinBox(mot->LastRowLastCol(), "sat yaw:", 0, 1, 0.1);
    sat_t = new DoubleSpinBox(mot->LastRowLastCol(), "sat thrust:", 0, 1, 0.1);
    
    km = new DoubleSpinBox(mot->NewRow(), "km:", -100, 100, 0.01, 6);
    km_z = new DoubleSpinBox(mot->LastRowLastCol(), "km_z:", -100, 100, 0.01, 6);
    
    m = new DoubleSpinBox(pos->NewRow(),"m",0,2000,0.001,3);
    g = new DoubleSpinBox(pos->LastRowLastCol(),"g",-10,10,0.01,3);
    
    //GroupBox *c_fisicas = new GroupBox(position->NewRow(), "Constantes Fisicas");
    
    t0 = double(GetTime())/1000000000;

    levant = Levant_diff("tanh", 8, 6, 3000);

    sgnpos_p << 0,0,0;
    sgnpos << 0,0,0;

    sgnori_p << 0,0,0;
    sgnori << 0,0,0;
    
    
}

Sliding_pos::~Sliding_pos(void) {}

void Sliding_pos::Reset(void) {
    first_update = true;
    t0 = 0;
    t0 = double(GetTime())/1000000000;
    sgnori_p << 0,0,0;
    sgnori << 0,0,0;

    levant.Reset();

    // sgnpos2 = Vector3ff(0,0,0);
    // sgn2 = Vector3ff(0,0,0);

    // sgnpos << 0,0,0;
    // sgn << 0,0,0;

    sgnpos_p << 0,0,0;
    sgnpos << 0,0,0;


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
    DataPlot1D *Sp = new DataPlot1D(position, "nu_rp", -5, 5);
    Sp->AddCurve(state->Element(7), DataPlot::Green);
    Sp->AddCurve(state->Element(8), DataPlot::Red);
    Sp->AddCurve(state->Element(9), DataPlot::Black);
    
}

void Sliding_pos::UseDefaultPlot9(const LayoutPosition *position) {    
    DataPlot1D *Sq = new DataPlot1D(position, "nu_r", -5, 5);
    Sq->AddCurve(state->Element(10), DataPlot::Green);
    Sq->AddCurve(state->Element(11), DataPlot::Red);
    Sq->AddCurve(state->Element(12), DataPlot::Black);
    
}


void Sliding_pos::UpdateFrom(const io_data *data) {
    float tactual=double(data->DataTime())/1000000000-t0;
    Printf("tactual: %f\n",tactual);
    float Trs=0, tau_roll=0, tau_pitch=0, tau_yaw=0, Tr=0;
    //Vector3ff Qe3 = Vector3ff(0,0,1);
    //Vector3ff ez = Vector3ff(0,0,-1);
    Eigen::Vector3f ez(0,0,1);
    
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

    //Vector3ff w = Vector3ff(input->ValueNoMutex(0, 6),input->ValueNoMutex(2, 6),input->ValueNoMutex(2, 6));

    Eigen::Vector3f xie(input->ValueNoMutex(0, 0),input->ValueNoMutex(1, 0),input->ValueNoMutex(2, 0));
    Eigen::Vector3f xiep(input->ValueNoMutex(0, 1),input->ValueNoMutex(1, 1),input->ValueNoMutex(2, 1));

    Eigen::Vector3f xid(input->ValueNoMutex(0, 2),input->ValueNoMutex(1, 2),input->ValueNoMutex(2, 2));
    Eigen::Vector3f xidpp(input->ValueNoMutex(0, 4),input->ValueNoMutex(1, 4),input->ValueNoMutex(2, 4));
    Eigen::Vector3f xidppp(input->ValueNoMutex(0, 5),input->ValueNoMutex(1, 5),input->ValueNoMutex(2, 5));

    Eigen::Vector3f w(input->ValueNoMutex(0, 6),input->ValueNoMutex(2, 6),input->ValueNoMutex(2, 6));

    Eigen::Quaternionf q(input->ValueNoMutex(0, 7),input->ValueNoMutex(1, 7),input->ValueNoMutex(2, 7),input->ValueNoMutex(3, 7));
    
    input->ReleaseMutex();

    Eigen::Vector3f alphap_v(alpha_x->Value(), alpha_y->Value(), alpha_z->Value());
    Eigen::Matrix3f alphap = alphap_v.asDiagonal();

    Eigen::Vector3f nup = xiep + alphap*xie;

    std::cout << "xiep = " << xiep << std::endl;
    std::cout << "xie = " << xie << std::endl;

    std::cout << "nup = " << nup << std::endl;

    sgnpos_p = nup.array().tanh();
    sgnpos = rk4_vec(sgnpos, sgnpos_p, delta_t);

    // sgnpos_p(0) = tanh(nup(0));
    // sgnpos_p(1) = tanh(nup(1));
    // sgnpos_p(2) = tanh(nup(2));

    // std::cout << "sgnpos_p = " << sgnpos_p << std::endl;

    // sgnpos(0) = rk4(function1d, sgnpos(0), sgnpos_p(0), delta_t);
    // sgnpos(1) = rk4(function1d, sgnpos(1), sgnpos_p(1), delta_t);
    // sgnpos(2) = rk4(function1d, sgnpos(2), sgnpos_p(2), delta_t);

    // std::cout << "sgnpos = " << sgnpos << std::endl;

    Eigen::Vector3f gammap_v(gamma_x->Value(), gamma_y->Value(), gamma_z->Value());
    Eigen::Matrix3f gammap = gammap_v.asDiagonal();

    Eigen::Vector3f nurp = nup + gammap*sgnpos;

    std::cout << "nurp = " << nurp << std::endl;


    Eigen::Vector3f xirpp = xidpp - alphap*xiep - gammap*sgnpos_p;

    Eigen::Vector3f Kpv(Kp_x->Value(), Kp_y->Value(), Kp_z->Value());
    Eigen::Matrix3f Kpm = Kpv.asDiagonal();


    Eigen::Vector3f u = -Kpm*nurp - m->Value()*g->Value()*ez + m->Value()*xirpp; //- m->Value()*g->Value()*ez + m->Value()*xirpp

    std::cout << "u = " << u << std::endl;


    Trs = u.norm();

    printf("Trs = %f\n", Trs);

    Eigen::Vector3f Qe3 = q.toRotationMatrix()*ez;
    // Eigen::Vector3f Qe3_(Qe3.x, Qe3.y, Qe3.z);
    // printf("Qe3 = %f %f %f\n", Qe3.x, Qe3.y, Qe3.z);
    //Vector3ff up = Vector3ff(0,0,0);
    
    Eigen::Vector3f Lambpv(sech(nup(0))*sech(nup(0)), sech(nup(1))*sech(nup(1)), sech(nup(2))*sech(nup(2)) );
    Eigen::Matrix3f Lambp = Lambpv.asDiagonal();

    // Eigen::Vector3f vec(sin(tactual), sin(tactual), sin(tactual));

    // float f = gammap->Value()*sin(alphap->Value()*tactual);
    // float alpha2 = Kp->Value();
    // float lamb = Kd->Value();

    // levant.setParam(alpha2, lamb);
    

    // ud = levant.Compute(f,delta_t);
    

    Eigen::Vector3f up = -(Kpm + m->Value()*alphap*I + m->Value()*gammap*Lambp) * (g->Value()*ez - (Trs/m->Value())*Qe3 - xidpp) - 
                        alphap*(Kpm+m->Value()*gammap*Lambp)*xiep - Kpm*gammap*sgnpos_p + m->Value()*xidppp;



    std::cout << "up = " << up << std::endl;

    Eigen::Vector3f un = u.normalized();
    Eigen::Vector3f upn = up.normalized();


    Eigen::Quaternionf qd( (0.5)*sqrt(-2*un(2)+2) , un(1)/sqrt(-2*un(2)+2), -un(0)/sqrt(-2*un(2)+2), 0);
    // qd.q0 = (0.5)*sqrt(-2*un(2)+2);
    // qd.q1 = un(1)/sqrt(-2*un(2)+2);
    // qd.q2 = -un(0)/sqrt(-2*un(2)+2);

    Eigen::Quaternionf qdp(-(0.5)*(upn(2)/sqrt(-2*un(2)+2)),
                            (upn(1)/sqrt(-2*un(2)+2)) + ((un(1)*upn(2))/pow(-2*un(2)+2,1.5)),
                           -(upn(0)/sqrt(-2*un(2)+2)) - ((un(0)*upn(2))/pow(-2*un(2)+2,1.5)),
                            0);
    // qdp.q0 = -(0.5)*(upn(2)/sqrt(-2*un(2)+2));
    // qdp.q1 = (upn(1)/sqrt(-2*un(2)+2)) + ((un(1)*upn(2))/pow(-2*un(2)+2,1.5));
    // qdp.q2 = -(upn(0)/sqrt(-2*un(2)+2)) - ((un(0)*upn(2))/pow(-2*un(2)+2,1.5));

    Eigen::Vector3f eta = qd.toRotationMatrix().eulerAngles(2, 1, 0);

    // printf("eta: %f %f %f\n",eta.roll,eta.pitch,eta.yaw);
    // printf("qd: %f %f %f %f\n",qd.q0,qd.q1,qd.q2,qd.q3);
    

    Eigen::Quaternionf qdc = qd.conjugate();
    Eigen::Quaternionf qe = q*qdc;

    Eigen::Vector3f wd(upn(1) - ( (un(1)*upn(2))/(1-un(2)) ), 
                      -upn(0) + ( (un(0)*upn(2))/(1-un(2)) ), 
                      (un(1)*upn(0) - un(0)*upn(1))/(1-un(2)));

    // wd.x = upn(1) - ( (un(1)*upn(2))/(1-un(2)) );
    // wd.y = -upn(0) + ( (un(0)*upn(2))/(1-un(2)) );
    // wd.z = (un(1)*upn(0) - un(0)*upn(1))/(1-un(2));

    Eigen::Vector3f we = w - wd;

    Eigen::Quaternionf QdTqe = qdc*qe*qd;
    Eigen::Vector3f QdTqe3(QdTqe.x(),QdTqe.y(),QdTqe.z());

    Eigen::Vector3f alphao_v(alpha_roll->Value(), alpha_pitch->Value(), alpha_yaw->Value());
    Eigen::Matrix3f alphao = alphao_v.asDiagonal();
    
    Eigen::Vector3f nu = we + alphao*QdTqe3;
    
    Eigen::Vector3f nu_t0 = 1*Eigen::Vector3f(1,1,1);
    
    Eigen::Vector3f nud = nu_t0*exp(-k->Value()*(tactual));
    
    Eigen::Vector3f nuq = nu-nud;

    //Printf("nuq = %f %f %f\n", nuq.x, nuq.y, nuq.z);

    sgnori_p = nuq.array().tanh();
    sgnori = rk4_vec(sgnori, sgnori_p, delta_t);
    
    // sgnori_p.x = signth(nuq.x,p->Value());
    // sgnori_p.y = signth(nuq.y,p->Value());
    // sgnori_p.z = signth(nuq.z,p->Value());
    
    // sgnori.x = rk4(function1d, sgnori.x, sgnori_p.x, delta_t);
    // sgnori.y = rk4(function1d, sgnori.y, sgnori_p.y, delta_t);
    // sgnori.z = rk4(function1d, sgnori.z, sgnori_p.z, delta_t);

    Eigen::Vector3f gammao_v(gamma_roll->Value(), gamma_pitch->Value(), gamma_yaw->Value());
    Eigen::Matrix3f gammao = gammao_v.asDiagonal();
    
    Eigen::Vector3f nur = nuq + gammao*sgnori;

    Eigen::Vector3f Kdv(Kd_roll->Value(), Kd_pitch->Value(), Kd_yaw->Value());
    Eigen::Matrix3f Kdm = Kdv.asDiagonal();
    
    Eigen::Vector3f tau = Kdm*nur;

    
    tau_roll = (float)tau(0)/km->Value();
    
    tau_pitch = (float)tau(1)/km->Value();
    
    tau_yaw = (float)tau(2)/km->Value();
    
    Tr = Trs/km_z->Value();
    
    tau_roll = Sat(tau_roll,sat_r->Value());
    tau_pitch = Sat(tau_pitch,sat_p->Value());
    tau_yaw = Sat(tau_yaw,sat_y->Value());
    Tr = -Sat(Tr,sat_t->Value());
    
    state->GetMutex();
    state->SetValueNoMutex(0, 0, tau_roll);
    state->SetValueNoMutex(1, 0, tau_pitch);
    state->SetValueNoMutex(2, 0, tau_yaw);
    state->SetValueNoMutex(3, 0, Tr);
    state->SetValueNoMutex(4, 0, eta.x());
    state->SetValueNoMutex(5, 0, eta.y());
    state->SetValueNoMutex(6, 0, eta.z());
    state->SetValueNoMutex(7, 0, nurp.x());
    state->SetValueNoMutex(8, 0, nurp.y());
    state->SetValueNoMutex(9, 0, nurp.z());
    state->SetValueNoMutex(10, 0, nur.x());
    state->SetValueNoMutex(11, 0, nur.y());
    state->SetValueNoMutex(12, 0, nur.z());
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
