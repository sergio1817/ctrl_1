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
#include "Rk4.h"
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

Sliding_pos::Sliding_pos(const LayoutPosition *position, string name): ControlLaw(position->getLayout(), name, 4){ // Salidas 4
    first_update = true;
    // init matrix
    input = new Matrix(this, 4, 8, floatType, name);

    MatrixDescriptor *desc = new MatrixDescriptor(7, 1);
    desc->SetElementName(0, 0, "u_roll");
    desc->SetElementName(1, 0, "u_pitch");
    desc->SetElementName(2, 0, "u_yaw");
    desc->SetElementName(3, 0, "u_z");
    desc->SetElementName(4, 0, "r");
    desc->SetElementName(5, 0, "p");
    desc->SetElementName(6, 0, "y");
    state = new Matrix(this, desc, floatType, name);
    delete desc;


    GroupBox *reglages_groupbox = new GroupBox(position, name);
    T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s", 0, 1, 0.001,6);
    gamma = new DoubleSpinBox(reglages_groupbox->NewRow(), "gamma:", -500, 500, 0.0001, 12);
    p = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "p:", 0, 50000, 1, 3);
    alpha = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "alpha:", 0, 50000, 0.5, 3);
    k = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "k:", 0, 50000, 0.5, 3);
    Kd = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "Kd:", 0, 50000, 0.5, 3);
    gammap = new DoubleSpinBox(reglages_groupbox->NewRow(), "gamma_p:", -500, 500, 0.0001, 12);
    alphap = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "alpha_p:", 0, 50000, 0.5, 3);
    Kp = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "Kp:", 0, 50000, 0.5, 3);
    sat_r = new DoubleSpinBox(reglages_groupbox->NewRow(), "sat roll:", 0, 1, 0.1);
    sat_p = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat pitch:", 0, 1, 0.1);
    sat_y = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat yaw:", 0, 1, 0.1);
    sat_t = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "sat thrust:", 0, 1, 0.1);
    
    km = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "km:", -100, 100, 0.01, 6);
    km_z = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "km_z:", -100, 100, 0.01, 6);
    
    m = new DoubleSpinBox(reglages_groupbox->NewRow(),"m",0,2000,0.001,3);
    g = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),"g",-10,10,0.01,3);
    
    //GroupBox *c_fisicas = new GroupBox(position->NewRow(), "Constantes Fisicas");
    
    t0 = double(GetTime())/1000000000;
    
    sgnp = Vector3Df(0,0,0);
    sgn = Vector3Df(0,0,0);

    sgnp2 = Vector3Df(0,0,0);
    sgn2 = Vector3Df(0,0,0);
    
    
}

Sliding_pos::~Sliding_pos(void) {}

void Sliding_pos::Reset(void) {
    first_update = true;
    t0 = 0;
    t0 = double(GetTime())/1000000000;
    sgnp = Vector3Df(0,0,0);
    sgn = Vector3Df(0,0,0);

    sgnp2 = Vector3Df(0,0,0);
    sgn2 = Vector3Df(0,0,0);
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


void Sliding_pos::UpdateFrom(const io_data *data) {
    float tactual=double(GetTime())/1000000000-t0;
    float Trs=0, tau_roll=0, tau_pitch=0, tau_yaw=0, Tr=0;
    Vector3Df Qe3 = Vector3Df(0,0,1);
    Vector3Df ez = Vector3Df(0,0,1);
    
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
    
    Vector3Df xie = Vector3Df(input->ValueNoMutex(0, 0),input->ValueNoMutex(1, 0),input->ValueNoMutex(2, 0));
    Vector3Df xiep = Vector3Df(input->ValueNoMutex(0, 1),input->ValueNoMutex(1, 1),input->ValueNoMutex(2, 1));

    Vector3Df xid = Vector3Df(input->ValueNoMutex(0, 2),input->ValueNoMutex(1, 2),input->ValueNoMutex(2, 2));
    Vector3Df xidpp = Vector3Df(input->ValueNoMutex(0, 4),input->ValueNoMutex(1, 4),input->ValueNoMutex(2, 4));
    Vector3Df xidppp = Vector3Df(input->ValueNoMutex(0, 5),input->ValueNoMutex(1, 5),input->ValueNoMutex(2, 5));

    Vector3Df w = Vector3Df(input->ValueNoMutex(0, 6),input->ValueNoMutex(2, 6),input->ValueNoMutex(2, 6));

    Quaternion q = Quaternion(input->ValueNoMutex(0, 7),input->ValueNoMutex(1, 7),input->ValueNoMutex(2, 7),input->ValueNoMutex(3, 7));
    
    input->ReleaseMutex();

    printf("xie = %f %f %f\n", xie.x, xie.y, xie.z);
    printf("xiep = %f %f %f\n", xiep.x, xiep.y, xiep.z);
    printf("xid = %f %f %f\n", xid.x, xid.y, xid.z);
    printf("xidpp = %f %f %f\n", xidpp.x, xidpp.y, xidpp.z);
    printf("xidppp = %f %f %f\n", xidppp.x, xidppp.y, xidppp.z);
    printf("w = %f %f %f\n", w.x, w.y, w.z);
    printf("q = %f %f %f %f\n", q.q0, q.q1, q.q2, q.q3);

    Vector3Df nup = xiep + alphap->Value()*xie;

    sgnp2.x = tanh(nup.x);
    sgnp2.y = tanh(nup.y);
    sgnp2.z = tanh(nup.z);

    sgn2.x = rk4(function1d, sgn2.x, sgnp2.x, delta_t);
    sgn2.y = rk4(function1d, sgn2.y, sgnp2.y, delta_t);
    sgn2.z = rk4(function1d, sgn2.z, sgnp2.z, delta_t);

    Vector3Df u = Vector3Df(0,0,0);
    // u.x = -(Kp->Value() + m->Value()*alphap->Value()) * xiep.x - (alphap->Value()*Kp->Value()) * xie.x - (Kp->Value()*gammap->Value()) * sgn2.x -
    //                     (m->Value()*gammap->Value()) * sgnp2.x - m->Value()*g->Value()*ez.x + m->Value()*xidpp.x;

    // u.y = -(Kp->Value() + m->Value()*alphap->Value()) * xiep.y - (alphap->Value()*Kp->Value()) * xie.y - (Kp->Value()*gammap->Value()) * sgn2.y -
    //                     (m->Value()*gammap->Value()) * sgnp2.y - m->Value()*g->Value()*ez.y + m->Value()*xidpp.y;

    // u.z = -(Kp->Value() + m->Value()*alphap->Value()) * xiep.z - (alphap->Value()*Kp->Value()) * xie.z - (Kp->Value()*gammap->Value()) * sgn2.z -
    //                     (m->Value()*gammap->Value()) * sgnp2.z - m->Value()*g->Value()*ez.z + m->Value()*xidpp.z;
    
    u.x = -(Kp->Value()) * (nup.x + gammap->Value()*sgn2.x);
    u.y = -(Kp->Value()) * (nup.y + gammap->Value()*sgn2.y);
    u.z = -(Kp->Value()) * (nup.z + gammap->Value()*sgn2.z);

    //u = -u;

    printf("u = %f %f %f\n", u.x, u.y, u.z);

    Trs = u.GetNorm();

    printf("Trs = %f\n", Trs);

    Qe3.Rotate(q);
    printf("Qe3 = %f %f %f\n", Qe3.x, Qe3.y, Qe3.z);
    Vector3Df up = Vector3Df(0,0,0);
    // up.x = -(Kp->Value() + m->Value()*alphap->Value() + m->Value()*gammap->Value()*pow(sech(nup.x),2)) * (  g->Value()*ez.x - (Trs/m->Value())*Qe3.x-xidpp.x) - 
    //         alphap->Value()*(Kp->Value()+m->Value()*gammap->Value()*pow(sech(nup.x),2))*xiep.x - Kp->Value()*gammap->Value()*sgnp2.x + m->Value()*xidppp.x;

    // up.y = -(Kp->Value() + m->Value()*alphap->Value() + m->Value()*gammap->Value()*pow(sech(nup.y),2)) * (  g->Value()*ez.y - (Trs/m->Value())*Qe3.y-xidpp.y) -
    //         alphap->Value()*(Kp->Value()+m->Value()*gammap->Value()*pow(sech(nup.y),2))*xiep.y - Kp->Value()*gammap->Value()*sgnp2.y + m->Value()*xidppp.y;

    // up.z = -(Kp->Value() + m->Value()*alphap->Value() + m->Value()*gammap->Value()*pow(sech(nup.z),2)) * (  g->Value()*ez.z - (Trs/m->Value())*Qe3.z-xidpp.z) -
    //         alphap->Value()*(Kp->Value()+m->Value()*gammap->Value()*pow(sech(nup.z),2))*xiep.z - Kp->Value()*gammap->Value()*sgnp2.z + m->Value()*xidppp.z;

    up.x = -Kp->Value() * ( g->Value()*ez.x - (Trs/m->Value())*Qe3.x - xidpp.x +  alphap->Value()*xiep.x + gammap->Value()*pow(sech(nup.x),2));
    up.y = -Kp->Value() * ( g->Value()*ez.y - (Trs/m->Value())*Qe3.y - xidpp.y +  alphap->Value()*xiep.y + gammap->Value()*pow(sech(nup.y),2));
    up.z = -Kp->Value() * ( g->Value()*ez.z - (Trs/m->Value())*Qe3.z - xidpp.z +  alphap->Value()*xiep.z + gammap->Value()*pow(sech(nup.z),2));

    //up = -up;

    printf("up = %f %f %f\n", up.x, up.y, up.z);

    Vector3Df un = u;
    Vector3Df upn = up;

    
    un.Normalize();
    upn.Normalize();

    printf("un = %f %f %f\n", un.x, un.y, un.z);
    printf("upn = %f %f %f\n", upn.x, upn.y, upn.z);
    printf("un_norm = %f\n", un.GetNorm());
    printf("upn_norm = %f\n", upn.GetNorm());

    Quaternion qd = Quaternion(0,0,0,0);
    qd.q0 = (0.5)*sqrt(-2*un.z+2);
    qd.q1 = un.y/sqrt(-2*un.z+2);
    qd.q2 = -un.x/sqrt(-2*un.z+2);

    Quaternion qdp = Quaternion(0,0,0,0);
    qdp.q0 = -(0.5)*(upn.z/sqrt(-2*un.z+2));
    qdp.q1 = (upn.y/sqrt(-2*un.y+2)) + ((un.y*upn.z)/pow(-2*un.z+2,1.5));
    qdp.q2 = -(upn.x/sqrt(-2*un.z+2)) - ((un.x*upn.z)/pow(-2*un.z+2,1.5));

    Euler eta = qd.ToEuler();

    printf("eta: %f %f %f\n",eta.roll,eta.pitch,eta.yaw);
    printf("qd: %f %f %f %f\n",qd.q0,qd.q1,qd.q2,qd.q3);
    
    Quaternion qdc = qd.GetConjugate();
    Quaternion qe = q*qdc;

    Vector3Df wd = Vector3Df(0,0,0);

    wd.x = upn.y - ( (un.y*upn.z)/(1-un.z) );
    wd.y = -upn.x + ( (un.x*upn.z)/(1-un.z) );
    wd.z = (un.y*upn.x - un.x*upn.y)/(1-un.z);

    Vector3Df we = w - wd;

    Quaternion QdTqe = qdc*qe*qd;
    Vector3Df QdTqe3 = Vector3Df(QdTqe.q1,QdTqe.q2,QdTqe.q3);
    
    Vector3Df nu = we + alpha->Value()*QdTqe3;
    
    Vector3Df nu_t0 = 0.01*Vector3Df(0,0,1);
    
    Vector3Df nud = nu_t0*exp(-k->Value()*(abs(tactual)));
    
    Vector3Df nuq = nu-nud;
    
    sgnp.x = signth(nuq.x,p->Value());
    sgnp.y = signth(nuq.y,p->Value());
    sgnp.z = signth(nuq.z,p->Value());
    
    sgn.x = rk4(function1d, sgn.x, sgnp.x, delta_t);
    sgn.y = rk4(function1d, sgn.y, sgnp.y, delta_t);
    sgn.z = rk4(function1d, sgn.z, sgnp.z, delta_t);
    
    Vector3Df nur = nuq + gamma->Value()*sgn;
    
    Vector3Df tau = Kd->Value()*nur;
    
    tau_roll = (float)tau.x/km->Value();
    
    tau_pitch = (float)tau.y/km->Value();
    
    tau_yaw = (float)tau.z/km->Value();
    
    Tr = sqrtf(Trs/km_z->Value());
    
    tau_roll = Sat(tau_roll,sat_r->Value());
    tau_pitch = Sat(tau_pitch,sat_p->Value());
    tau_yaw = Sat(tau_yaw,sat_y->Value());
    Tr = -Sat(Tr,sat_t->Value());
    
    state->GetMutex();
    state->SetValueNoMutex(0, 0, tau_roll);
    state->SetValueNoMutex(1, 0, tau_pitch);
    state->SetValueNoMutex(2, 0, tau_yaw);
    state->SetValueNoMutex(3, 0, Tr);
    state->SetValueNoMutex(4, 0, eta.roll);
    state->SetValueNoMutex(5, 0, eta.pitch);
    state->SetValueNoMutex(6, 0, eta.yaw);
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
    return 1 / cosh(value);
}

} // end namespace filter
} // end namespace flair
