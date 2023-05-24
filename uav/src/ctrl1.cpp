//  created:    2011/05/01  /   2022/11/20
//  filename:   ctrl1.cpp
//
//  author:     Guillaume Sanahuja / Sergio Urzua
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Proyecto 2022
//
//
/*********************************************************************/

#include "ctrl1.h"
#include "Sliding.h"
#include "Sliding_pos.h"
#include "TargetJR3.h"
//#include "MetaJR3.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <DoubleSpinBox.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <Label.h>
#include <FrameworkManager.h>
#include <Matrix.h>
#include <MatrixDescriptor.h>
#include <cmath>
#include <Tab.h>
#include <Ahrs.h>
#include <AhrsData.h>
#include <ComboBox.h>
#include <GroupBox.h>
#include <TabWidget.h>
#include <Tab.h>

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;


ctrl1::ctrl1(TargetController *controller, TargetJR3 *jr3): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false), jr3(jr3) {
    Uav* uav=GetUav();

    VrpnClient* vrpnclient=new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(),80,uav->GetDefaultVrpnConnectionType());
    //VrpnClient* vrpnclient=new VrpnClient("vrpn", "192.168.147.197:3883",80,uav->GetDefaultVrpnConnectionType());
    
    if(vrpnclient->ConnectionType()==VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(),(uint8_t)0);
        //targetVrpn=new MetaVrpnObject("target",1);
    } else if (vrpnclient->ConnectionType()==VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        //targetVrpn=new MetaVrpnObject("target");
    } else if (vrpnclient->ConnectionType()==VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        //targetVrpn=new MetaVrpnObject("target");
    }
    
    
    jr3->Start();
    getFrameworkManager()->AddDeviceToLog(jr3);

    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    //getFrameworkManager()->AddDeviceToLog(targetVrpn);
    vrpnclient->Start();
    
    GroupBox *groupbox = new GroupBox(GetButtonsLayout()->NewRow(), "Controles");
    
    
    Tab *lawTab2 = new Tab(getFrameworkManager()->GetTabWidget(), "control laws custom");
    TabWidget *tabWidget2 = new TabWidget(lawTab2->NewRow(), "laws");
    
    setupLawTab2 = new Tab(tabWidget2, "Setup Sliding");
    setupLawTab3 = new Tab(tabWidget2, "Setup Sliding Pos");
    graphLawTab2 = new Tab(tabWidget2, "Graficas Sliding");
    graphLawTab3 = new Tab(tabWidget2, "Graficas Sliding Pos");

    Tab *posTab = new Tab(getFrameworkManager()->GetTabWidget(), "position");
    TabWidget *Pos_tabWidget = new TabWidget(posTab->NewRow(), "position");

    positionTab = new Tab(Pos_tabWidget, "Setup position");
    positiongTab = new Tab(Pos_tabWidget, "Graph position");

    GroupBox *posbox = new GroupBox(positionTab->At(0,0), "position");
    
    xd = new DoubleSpinBox(posbox->NewRow(), "x", " m", -5, 5, 0.001, 3);
    yd = new DoubleSpinBox(posbox->LastRowLastCol(), "y", " m", -5, 5, 0.001, 3);
    zd = new DoubleSpinBox(posbox->LastRowLastCol(), "z", " m", -5, 5, 0.001, 3);

    xdp = new DoubleSpinBox(posbox->NewRow(), "xp", " m/s", -5, 5, 0.001, 3);
    ydp = new DoubleSpinBox(posbox->LastRowLastCol(), "yp", " m/s", -5, 5, 0.001, 3);
    zdp = new DoubleSpinBox(posbox->LastRowLastCol(), "zp", " m/s", -5, 5, 0.001, 3);

    xdpp = new DoubleSpinBox(posbox->NewRow(), "xpp", " m/s^2", -5, 5, 0.001, 3);
    ydpp = new DoubleSpinBox(posbox->LastRowLastCol(), "ypp", " m/s^2", -5, 5, 0.001, 3);
    zdpp = new DoubleSpinBox(posbox->LastRowLastCol(), "zpp", " m/s^2", -5, 5, 0.001, 3);

    xdppp = new DoubleSpinBox(posbox->NewRow(), "xppp", " m/s^3", -5, 5, 0.001, 3);
    ydppp = new DoubleSpinBox(posbox->LastRowLastCol(), "yppp", " m/s^3", -5, 5, 0.001, 3);
    zdppp = new DoubleSpinBox(posbox->LastRowLastCol(), "zppp", " m/s^3", -5, 5, 0.001, 3);
    
    control_select=new ComboBox(groupbox->NewRow(),"select control");
    control_select->AddItem("Sliding");
    control_select->AddItem("Sliding Pos");
    
    l2 = new Label(groupbox->LastRowLastCol(), "Control selec");
    l2->SetText("Control: off");
    
    start_prueba1=new PushButton(groupbox->NewRow(),"start control");
    stop_prueba1=new PushButton(groupbox->NewRow(),"stop control");
    
    
    u_sliding = new Sliding(setupLawTab2->At(0, 0), "u_smc");
    u_sliding->UseDefaultPlot(graphLawTab2->At(0, 0));
    u_sliding->UseDefaultPlot2(graphLawTab2->At(0, 1));
    u_sliding->UseDefaultPlot3(graphLawTab2->At(0, 2));
    u_sliding->UseDefaultPlot4(graphLawTab2->At(1, 2));

    u_sliding_pos = new Sliding_pos(setupLawTab3->At(0, 0), "u_smc_pos");
    u_sliding_pos->UseDefaultPlot(graphLawTab3->At(0, 0));
    u_sliding_pos->UseDefaultPlot2(graphLawTab3->At(0, 1));
    u_sliding_pos->UseDefaultPlot3(graphLawTab3->At(0, 2));
    u_sliding_pos->UseDefaultPlot4(graphLawTab3->At(1, 2));

    u_sliding_pos->UseDefaultPlot8(graphLawTab3->At(1, 0));
    u_sliding_pos->UseDefaultPlot9(graphLawTab3->At(1, 1));

    u_sliding_pos->UseDefaultPlot5(positiongTab->At(0, 0));
    u_sliding_pos->UseDefaultPlot6(positiongTab->At(0, 1));
    u_sliding_pos->UseDefaultPlot7(positiongTab->At(0, 2));
    
    
    customOrientation=new AhrsData(this,"orientation");

    //getFrameworkManager()->AddDeviceToLog(u_sliding);
    //AddDeviceToControlLawLog(u_sliding);


}

ctrl1::~ctrl1() {
}

//this method is called by UavStateMachine::Run (main loop) when TorqueMode is Custom
void ctrl1::ComputeCustomTorques(Euler &torques) {
    ComputeDefaultTorques(torques);
    thrust = ComputeDefaultThrust();
    switch(control_select->CurrentIndex()) {
        case 0:
            Printf("Fx:%f Fy:%f Fz:%f\n",jr3->GetFx(),jr3->GetFy(),jr3->GetFz());
            sliding_ctrl(torques);
            break;
        
        case 1:
            sliding_ctrl_pos(torques);
            break;
            
    }
    
}

float ctrl1::ComputeCustomThrust(void) {
    return thrust;
}

void ctrl1::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::control))) {
        // if (!targetVrpn->IsTracked(500)) {
        //     Thread::Err("VRPN, target lost\n");
        //     vrpnLost=true;
        //     EnterFailSafeMode();
        //     Land();
        // }
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
    }
}

// const AhrsData *ctrl1::GetOrientation(void) const {
//     //get yaw from vrpn
// 		Quaternion vrpnQuaternion;
//     uavVrpn->GetQuaternion(vrpnQuaternion);

//     //get roll, pitch and w from imu
//     Quaternion ahrsQuaternion;
//     Vector3Df ahrsAngularSpeed;
//     GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

//     Euler ahrsEuler=ahrsQuaternion.ToEuler();
//     ahrsEuler.yaw=vrpnQuaternion.ToEuler().yaw;
//     Quaternion mixQuaternion=ahrsEuler.ToQuaternion();

//     customOrientation->SetQuaternionAndAngularRates(mixQuaternion,ahrsAngularSpeed);

//     return customOrientation;
// }



void ctrl1::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode=BehaviourMode_t::Default;
        vrpnLost=false;
        break;
    case Event_t::Stopped:
        l2->SetText("Control: off");
        control_select->setEnabled(true);
        //first_update==true;
        behaviourMode=BehaviourMode_t::Default;
        break;
    case Event_t::EnteringFailSafeMode:
        l2->SetText("Control: off");
        control_select->setEnabled(true);
        //first_update==true;
        behaviourMode=BehaviourMode_t::Default;
        break;
    }
}


void ctrl1::ExtraCheckPushButton(void) {
    if(start_prueba1->Clicked() && (behaviourMode!=BehaviourMode_t::control)) {
        Startctrl1();
    }

    if(stop_prueba1->Clicked() && (behaviourMode==BehaviourMode_t::control)) {
        Stopctrl1();
    }
}

void ctrl1::ExtraCheckJoystick(void) {
    //R1
    if(GetTargetController()->IsButtonPressed(9) && (behaviourMode!=BehaviourMode_t::control)) {
        Startctrl1();
    }
    //L1
    if(GetTargetController()->IsButtonPressed(6) && (behaviourMode==BehaviourMode_t::control)) {
        Stopctrl1();
    }
    
}


void ctrl1::Startctrl1(void) {
    control_select->setEnabled(false);
    //ask UavStateMachine to enter in custom torques
    if (SetTorqueMode(TorqueMode_t::Custom) && SetThrustMode(ThrustMode_t::Custom)) {
        Thread::Info("ctrl1: start\n");
        u_sliding->Reset();
        u_sliding_pos->Reset();
    } else {
        Thread::Warn("ctrl1: could not start\n");
        l2->SetText("Control: err");
        control_select->setEnabled(true);
        return;
    }
    switch(control_select->CurrentIndex()) {
        case 0:
            l2->SetText("Control: Sliding");
            Thread::Info("Sliding\n");
            break;
        
        case 1:
            l2->SetText("Control: Sliding pos");
            Thread::Info("Sliding pos\n");
            //l2->SetText("Control: Nested");
            //Thread::Info("Nested\n");
            break;
            
    }

    behaviourMode=BehaviourMode_t::control;
}

void ctrl1::Stopctrl1(void) {
    control_select->setEnabled(true);
    //just ask to enter fail safe mode
    l2->SetText("Control: off");
    //first_update==true;
    SetTorqueMode(TorqueMode_t::Default);
    SetThrustMode(ThrustMode_t::Default);
    behaviourMode=BehaviourMode_t::Default;
    EnterFailSafeMode();
}



void ctrl1::sliding_ctrl(Euler &torques){
    float ti = double(GetTime());
    const AhrsData *refOrientation = GetDefaultReferenceOrientation();
    Quaternion refQuaternion;
    Vector3Df refAngularRates;
    refOrientation->GetQuaternionAndAngularRates(refQuaternion, refAngularRates);
    float tf = (GetTime()-ti)/1000000000;

    printf("ref: %f\n", tf);

    ti = double(GetTime());
    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    tf = (GetTime()-ti)/1000000000;

    printf("cur: %f\n", tf);
    
    //Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    
    float refAltitude, refVerticalVelocity;
    GetDefaultReferenceAltitude(refAltitude, refVerticalVelocity);
    
    float z, zp;
    
    AltitudeValues(z,zp);
    
    float ze = z - refAltitude;
    
    Vector3Df we = currentAngularRates - refAngularRates;
    
    u_sliding->SetValues(ze,zp,we,currentQuaternion,refQuaternion);
    
    u_sliding->Update(GetTime());
    
    //Thread::Info("%f\t %f\t %f\t %f\n",u_sliding->Output(0),u_sliding->Output(1), u_sliding->Output(2), u_sliding->Output(3));
    
    torques.roll = u_sliding->Output(0);
    torques.pitch = u_sliding->Output(1);
    torques.yaw = u_sliding->Output(2);
    thrust = u_sliding->Output(3);
    //thrust = ComputeDefaultThrust();
    

}

void ctrl1::sliding_ctrl_pos(Euler &torques){
    float tactual=double(GetTime())/1000000000-u_sliding_pos->t0;
    //printf("t: %f\n",tactual);


    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Quaternion uav_quat;

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    uavVrpn->GetQuaternion(uav_quat);

    //Thread::Info("Pos: %f\t %f\t %f\n",uav_pos.x,uav_pos.y, uav_pos.z);
    Printf("Pos: %f\t %f\t %f\n",uav_pos.x,uav_pos.y, uav_pos.z);
    Printf("Vel: %f\t %f\t %f\n",uav_vel.x,uav_vel.y, uav_vel.z);
    //Thread::Info("Vel: %f\t %f\t %f\n",uav_vel.x,uav_vel.y, uav_vel.z);

    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    
    double a = xd->Value(); // 0.1
    double az = yd->Value(); // 0.001
    double b = zd->Value(); // 0.01

    // Vector3Df xid = Vector3Df(a*sin(b*tactual),a*cos(b*tactual),az*sin(b*tactual)-2);
    // Vector3Df xidp = Vector3Df(a*cos(b*tactual),-a*sin(b*tactual),az*cos(b*tactual));
    // Vector3Df xidpp = Vector3Df(-a*sin(b*tactual),-a*cos(b*tactual),-az*sin(b*tactual));
    // Vector3Df xidppp = Vector3Df(-a*cos(b*tactual),a*sin(b*tactual),-az*cos(b*tactual));

    Vector3Df xid = Vector3Df(xd->Value(),yd->Value(),zd->Value());
    Vector3Df xidp = Vector3Df(0,0,0);
    Vector3Df xidpp = Vector3Df(0,0,0);
    Vector3Df xidppp = Vector3Df(0,0,0);

    printf("xid: %f\t %f\t %f\n",xid.x,xid.y, xid.z);
    
    u_sliding_pos->SetValues(uav_pos-xid,uav_vel-xidp,xid,xidpp,xidppp,currentAngularSpeed,currentQuaternion);
    
    u_sliding_pos->Update(GetTime());
    
    //Thread::Info("%f\t %f\t %f\t %f\n",u_sliding->Output(0),u_sliding->Output(1), u_sliding->Output(2), u_sliding->Output(3));
    
    if(xdpp->Value() == 0){
        torques.roll = u_sliding_pos->Output(0);
        torques.pitch = u_sliding_pos->Output(1);
        torques.yaw = u_sliding_pos->Output(2);
        thrust = u_sliding_pos->Output(3);
    }
    //thrust = ComputeDefaultThrust();
    

}
