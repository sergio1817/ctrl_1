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
#include "Sliding_force.h"
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

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;


ctrl1::ctrl1(TargetController *controller): UavStateMachine(controller), behaviourMode(BehaviourMode_t::Default), vrpnLost(false) {
    Uav* uav=GetUav();

    std::string ip_dir;
    if(uav->GetType()=="x4_simu"){
        ip_dir = uav->GetDefaultVrpnAddress();
    } else {
        ip_dir = "192.168.147.103:3883";
    }
    //ip_dir = "192.168.147.103:3883";
    VrpnClient* vrpnclient=new VrpnClient("vrpn", ip_dir,80,uav->GetDefaultVrpnConnectionType());
    
    
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
    
    //getFrameworkManager()->AddDeviceToLog(jr3);
    //jr3->Start();

    //set vrpn as failsafe altitude sensor for mamboedu as us in not working well for the moment
    if(uav->GetType()=="mamboedu") {
        SetFailSafeAltitudeSensor(uavVrpn->GetAltitudeSensor());
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2),DataPlot::Green);

    GroupBox *groupbox = new GroupBox(GetButtonsLayout()->NewRow(), "Controles");
    
    
    Tab *lawTab2 = new Tab(getFrameworkManager()->GetTabWidget(), "control laws custom");
    TabWidget *tabWidget2 = new TabWidget(lawTab2->NewRow(), "laws");
    
    setupLawTab2 = new Tab(tabWidget2, "Setup Sliding");
    setupLawTab3 = new Tab(tabWidget2, "Setup Sliding Pos");
    graphLawTab2 = new Tab(tabWidget2, "Graficas Sliding");
    graphLawTab3 = new Tab(tabWidget2, "Graficas Sliding Pos");

    //Tab *posTab = new Tab(getFrameworkManager()->GetTabWidget(), "position");
    //TabWidget *Pos_tabWidget = new TabWidget(posTab->NewRow(), "position");

    positionTab = new Tab(tabWidget2, "Reference position");
    positiongTab = new Tab(tabWidget2, "Desired orientation");

    GroupBox *posbox = new GroupBox(positionTab->NewRow(), "Setup reference");
    GroupBox *regbox = new GroupBox(positionTab->NewRow(), "Setup regulation");
    GroupBox *trackbox = new GroupBox(positionTab->NewRow(), "Setup tracking");
    GroupBox *trajbox = new GroupBox(positionTab->NewRow(), "Setup trajectory");

    position_behavior = new ComboBox(posbox->NewRow(),"Select behavior");
    position_behavior->AddItem("Regulation");
    position_behavior->AddItem("Tracking");
    position_behavior->AddItem("Trajectory");

    xd_behavior = new ComboBox(trackbox->NewRow(),"Select xd behavior");
    xd_behavior->AddItem("Regulation");
    xd_behavior->AddItem("Sin");
    xd_behavior->AddItem("Cos");

    yd_behavior = new ComboBox(trackbox->LastRowLastCol(),"Select yd behavior");
    yd_behavior->AddItem("Regulation");
    yd_behavior->AddItem("Sin");
    yd_behavior->AddItem("Cos");

    zd_behavior = new ComboBox(trackbox->LastRowLastCol(),"Select zd behavior");
    zd_behavior->AddItem("Regulation");
    zd_behavior->AddItem("Sin");
    zd_behavior->AddItem("Cos");

    GroupBox *xdkbox = new GroupBox(trackbox->NewRow(), "xd");
    GroupBox *ydkbox = new GroupBox(trackbox->LastRowLastCol(), "yd");
    GroupBox *zdkbox = new GroupBox(trackbox->LastRowLastCol(), "zd");
    
    xd = new DoubleSpinBox(regbox->NewRow(), "x", " m", -2, 2, 0.1, 2);
    yd = new DoubleSpinBox(regbox->LastRowLastCol(), "y", " m", -2, 2, 0.1, 2);
    zd = new DoubleSpinBox(regbox->LastRowLastCol(), "z", " m", -2, 2, 0.1, 2);
    
    lx = new Label(xdkbox->NewRow(), "funcion");
    lx->SetText("a*fnc(w*t) + b");
    ax = new DoubleSpinBox(xdkbox->NewRow(), "Amplitude (a)", -2, 2, 0.1, 2);
    wx = new DoubleSpinBox(xdkbox->NewRow(), "Frecuency (w)", 0, 10, 0.1, 2);
    bx = new DoubleSpinBox(xdkbox->NewRow(), "Offset (b)", 0, 3, 0.1, 2);

    ly = new Label(ydkbox->NewRow(), "funcion");
    ly->SetText("a*fnc(w*t) + b");
    ay = new DoubleSpinBox(ydkbox->NewRow(), "Amplitude (a)", -2, 2, 0.1, 2);
    wy = new DoubleSpinBox(ydkbox->NewRow(), "Frecuency (w)", 0, 10, 0.1, 2);
    by = new DoubleSpinBox(ydkbox->NewRow(), "Offset (b)", 0, 3, 0.1, 2);

    lz = new Label(zdkbox->NewRow(), "funcion");
    lz->SetText("a*fnc(w*t) + b");
    az = new DoubleSpinBox(zdkbox->NewRow(), "Amplitude (a)", -2, 2, 0.1, 2);
    wz = new DoubleSpinBox(zdkbox->NewRow(), "Frecuency (w)", 0, 10, 0.1, 2);
    bz = new DoubleSpinBox(zdkbox->NewRow(), "Offset (b)", -3, 0, 0.1, 2);

    
    
    control_select=new ComboBox(groupbox->NewRow(),"select control");
    control_select->AddItem("Sliding");
    control_select->AddItem("Sliding Pos");
    control_select->AddItem("Sliding Force-Position");
    
    l2 = new Label(groupbox->LastRowLastCol(), "Control selec");
    l2->SetText("Control: off");
    
    start_prueba1=new PushButton(groupbox->NewRow(),"start control");
    stop_prueba1=new PushButton(groupbox->NewRow(),"stop control");
    
    
    u_sliding = new Sliding(setupLawTab2->At(0, 0), "u_smc");
    u_sliding->UseDefaultPlot(graphLawTab2->At(0, 0));
    u_sliding->UseDefaultPlot2(graphLawTab2->At(0, 1));
    u_sliding->UseDefaultPlot3(graphLawTab2->At(0, 2));
    u_sliding->UseDefaultPlot4(graphLawTab2->At(1, 2));
    u_sliding->UseDefaultPlot5(graphLawTab2->At(1, 0));

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


    Tab *lawTab3 = new Tab(getFrameworkManager()->GetTabWidget(), "Force");
    TabWidget *tabWidget3 = new TabWidget(lawTab3->NewRow(), "laws");
    
    Tab *setupForceTab = new Tab(tabWidget3, "Setup");
    Tab *slidingTab = new Tab(tabWidget3, "Sliding");
    Tab *errorsTab = new Tab(tabWidget3, "Graphs");
    Tab *ctrlfTab = new Tab(tabWidget3, "Control");
    Tab *outputTab = new Tab(tabWidget3, "Outputs");

    Tab *ref = new Tab(tabWidget3, "Reference force");

    GroupBox *forcebox = new GroupBox(ref->NewRow(), "Setup reference");
    GroupBox *regboxf = new GroupBox(ref->NewRow(), "Setup regulation");
    GroupBox *trackboxf = new GroupBox(ref->NewRow(), "Setup tracking");
    GroupBox *trajboxf = new GroupBox(ref->NewRow(), "Setup trajectory");

    force_behavior = new ComboBox(forcebox->NewRow(),"Select behavior");
    force_behavior->AddItem("Regulation");
    force_behavior->AddItem("Tracking");
    force_behavior->AddItem("Trajectory");

    fx_behavior = new ComboBox(trackboxf->NewRow(),"Select fxd behavior");
    fx_behavior->AddItem("Regulation");
    fx_behavior->AddItem("Sin");
    fx_behavior->AddItem("Cos");

    fy_behavior = new ComboBox(trackboxf->LastRowLastCol(),"Select fyd behavior");
    fy_behavior->AddItem("Regulation");
    fy_behavior->AddItem("Sin");
    fy_behavior->AddItem("Cos");

    fz_behavior = new ComboBox(trackboxf->LastRowLastCol(),"Select fzd behavior");
    fz_behavior->AddItem("Regulation");
    fz_behavior->AddItem("Sin");
    fz_behavior->AddItem("Cos");

    GroupBox *fxkbox = new GroupBox(trackboxf->NewRow(), "fxd");
    GroupBox *fykbox = new GroupBox(trackboxf->LastRowLastCol(), "fyd");
    GroupBox *fzkbox = new GroupBox(trackboxf->LastRowLastCol(), "fzd");
    
    fxd = new DoubleSpinBox(regboxf->NewRow(), "fx", " N", -10, 10, 0.1, 2);
    fyd = new DoubleSpinBox(regboxf->LastRowLastCol(), "fy", " N", -10, 10, 0.1, 2);
    fzd = new DoubleSpinBox(regboxf->LastRowLastCol(), "fz", " N", -10, 10, 0.1, 2);
    
    lfx = new Label(fxkbox->NewRow(), "funcion");
    lfx->SetText("a*fnc(w*t) + b");
    afx = new DoubleSpinBox(fxkbox->NewRow(), "Amplitude (a)", -2, 2, 0.1, 2);
    wfx = new DoubleSpinBox(fxkbox->NewRow(), "Frecuency (w)", 0, 10, 0.1, 2);
    bfx = new DoubleSpinBox(fxkbox->NewRow(), "Offset (b)", 0, 3, 0.1, 2);

    lfy = new Label(fykbox->NewRow(), "funcion");
    lfy->SetText("a*fnc(w*t) + b");
    afy = new DoubleSpinBox(fykbox->NewRow(), "Amplitude (a)", -2, 2, 0.1, 2);
    wfy = new DoubleSpinBox(fykbox->NewRow(), "Frecuency (w)", 0, 10, 0.1, 2);
    bfy = new DoubleSpinBox(fykbox->NewRow(), "Offset (b)", 0, 3, 0.1, 2);

    lfz = new Label(fzkbox->NewRow(), "funcion");
    lfz->SetText("a*fnc(w*t) + b");
    afz = new DoubleSpinBox(fzkbox->NewRow(), "Amplitude (a)", -2, 2, 0.1, 2);
    wfz = new DoubleSpinBox(fzkbox->NewRow(), "Frecuency (w)", 0, 10, 0.1, 2);
    bfz = new DoubleSpinBox(fzkbox->NewRow(), "Offset (b)", -3, 0, 0.1, 2);

    u_sliding_force = new Sliding_force(setupForceTab->At(0, 0), "u_smc_force");
    u_sliding_force->UseDefaultPlot(outputTab->At(0, 0));
    u_sliding_force->UseDefaultPlot2(outputTab->At(0, 1));
    u_sliding_force->UseDefaultPlot3(outputTab->At(0, 2));
    u_sliding_force->UseDefaultPlot4(outputTab->At(1, 2));

    u_sliding_force->UseDefaultPlot8(slidingTab->At(0, 0));
    u_sliding_force->UseDefaultPlot9(slidingTab->At(0, 1));
    u_sliding_force->UseDefaultPlot10(slidingTab->At(0, 2));
    u_sliding_force->UseDefaultPlot11(slidingTab->At(1, 2));

    u_sliding_force->UseDefaultPlot5(errorsTab->At(1, 0));
    u_sliding_force->UseDefaultPlot6(errorsTab->At(1, 1));
    u_sliding_force->UseDefaultPlot7(errorsTab->At(1, 2));

    u_sliding_force->UseDefaultPlot12(errorsTab->At(0, 0));
    u_sliding_force->UseDefaultPlot13(errorsTab->At(0, 1));
    u_sliding_force->UseDefaultPlot14(errorsTab->At(0, 2));

    u_sliding_force->UseDefaultPlot15(ctrlfTab->At(0, 0));
    u_sliding_force->UseDefaultPlot16(ctrlfTab->At(0, 1));
    u_sliding_force->UseDefaultPlot17(ctrlfTab->At(0, 2));

    u_sliding_force->UseDefaultPlot18(ctrlfTab->At(1, 0));
    u_sliding_force->UseDefaultPlot19(ctrlfTab->At(1, 1));
    u_sliding_force->UseDefaultPlot20(ctrlfTab->At(1, 2));

    u_sliding_force->UseDefaultPlot22(ctrlfTab->At(2, 0));
    u_sliding_force->UseDefaultPlot21(ctrlfTab->At(2, 2));


    //getFrameworkManager()->AddDeviceToLog(u_sliding);
    AddDeviceToControlLawLog(u_sliding);
    AddDeviceToControlLawLog(u_sliding_pos);
    AddDeviceToControlLawLog(u_sliding_force);


}

ctrl1::~ctrl1() {
}

//this method is called by UavStateMachine::Run (main loop) when TorqueMode is Custom
void ctrl1::ComputeCustomTorques(Euler &torques) {
    ComputeDefaultTorques(torques);
    thrust = ComputeDefaultThrust();
    switch(control_select->CurrentIndex()) {
        case 0:
            //Printf("Fx:%f Fy:%f Fz:%f\n",jr3->GetFx(),jr3->GetFy(),jr3->GetFz());
            sliding_ctrl(torques);
            break;
        
        case 1:
            if(vrpnLost==true){
                Thread::Err("Posrition control can't start: VRPN lost\n");
            } else {
                sliding_ctrl_pos(torques);
            }
            break;
        
        // case 2:
        //     if(vrpnLost==true){
        //         Thread::Err("Force-position control can't start: VRPN lost or JR3 disconnected\n");
        //     } else {
        //         sliding_ctrl_force(torques);
        //     }
        //     break;
    }
    
}

float ctrl1::ComputeCustomThrust(void) {
    return thrust;
}

void ctrl1::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode==BehaviourMode_t::control))) {
        if (!targetVrpn->IsTracked(500)) {
            Thread::Err("VRPN, target lost\n");
            vrpnLost=true;
            EnterFailSafeMode();
            Land();
        }
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
        vrpnLost=false;
        behaviourMode=BehaviourMode_t::Default;
        break;
    case Event_t::EnteringFailSafeMode:
        l2->SetText("Control: off");
        control_select->setEnabled(true);
        //first_update==true;
        vrpnLost=false;
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
        //u_sliding_force->Reset();
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
            break;
        
        // case 2:
        //     l2->SetText("Control: Sliding force-position");
        //     Thread::Info("Sliding force-position\n");
        //     break;
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

void ctrl1::pos_reference(Vector3Df &xid, Vector3Df &xidp, Vector3Df &xidpp, Vector3Df &xidppp, float tactual){
    
    switch(position_behavior->CurrentIndex()){
    case 0:
        // regulation
        xid = Vector3Df(xd->Value(),yd->Value(),zd->Value());
        xidp = Vector3Df(0,0,0);
        xidpp = Vector3Df(0,0,0);
        xidppp = Vector3Df(0,0,0);
        break;
    
    case 1:
        // tracking
        switch(xd_behavior->CurrentIndex()){
        case 0:
            // regulation
            xid.x = xd->Value();
            xidp.x = 0;
            xidpp.x = 0;
            xidppp.x = 0;
            break;
        case 1:
            // sin
            xid.x = ax->Value()*sin(wx->Value()*tactual)+bx->Value();
            xidp.x = ax->Value()*wx->Value()*cos(wx->Value()*tactual);
            xidpp.x = -ax->Value()*wx->Value()*wx->Value()*sin(wx->Value()*tactual);
            xidppp.x = -ax->Value()*wx->Value()*wx->Value()*wx->Value()*cos(wx->Value()*tactual);
            break;
        case 2:
            // cos
            xid.x = ax->Value()*cos(wx->Value()*tactual)+bx->Value();
            xidp.x = -ax->Value()*wx->Value()*sin(wx->Value()*tactual);
            xidpp.x = -ax->Value()*wx->Value()*wx->Value()*cos(wx->Value()*tactual);
            xidppp.x = ax->Value()*wx->Value()*wx->Value()*wx->Value()*sin(wx->Value()*tactual);
            break;
        }

        switch(yd_behavior->CurrentIndex()){
        case 0:
            // regulation
            xid.y = yd->Value();
            xidp.y = 0;
            xidpp.y = 0;
            xidppp.y = 0;
            break;
        case 1:
            // sin
            xid.y = ay->Value()*sin(wy->Value()*tactual)+by->Value();
            xidp.y = ay->Value()*wy->Value()*cos(wy->Value()*tactual);
            xidpp.y = -ay->Value()*wy->Value()*wy->Value()*sin(wy->Value()*tactual);
            xidppp.y = -ay->Value()*wy->Value()*wy->Value()*wy->Value()*cos(wy->Value()*tactual);
            break;
        case 2:
            // cos
            xid.y = ay->Value()*cos(wy->Value()*tactual)+by->Value();
            xidp.y = -ay->Value()*wy->Value()*sin(wy->Value()*tactual);
            xidpp.y = -ay->Value()*wy->Value()*wy->Value()*cos(wy->Value()*tactual);
            xidppp.y = ay->Value()*wy->Value()*wy->Value()*wy->Value()*sin(wy->Value()*tactual);
            break;
        }

        switch(zd_behavior->CurrentIndex()){
        case 0:
            // regulation
            xid.z = zd->Value();
            xidp.z = 0;
            xidpp.z = 0;
            xidppp.z = 0;
            break;
        case 1:
            // sin
            xid.z = az->Value()*sin(wz->Value()*tactual)+bz->Value();
            xidp.z = az->Value()*wz->Value()*cos(wz->Value()*tactual);
            xidpp.z = -az->Value()*wz->Value()*wz->Value()*sin(wz->Value()*tactual);
            xidppp.z = -az->Value()*wz->Value()*wz->Value()*wz->Value()*cos(wz->Value()*tactual);
            break;
        case 2:
            // cos
            xid.z = az->Value()*cos(wz->Value()*tactual)+bz->Value();
            xidp.z = -az->Value()*wz->Value()*sin(wz->Value()*tactual);
            xidpp.z = -az->Value()*wz->Value()*wz->Value()*cos(wz->Value()*tactual);
            xidppp.z = az->Value()*wz->Value()*wz->Value()*wz->Value()*sin(wz->Value()*tactual);
            break;
        }
        break;
    
    case 2:
        // trajectory
        break;
    default:
        xid = Vector3Df(0,0,0);
        xidp = Vector3Df(0,0,0);
        xidpp = Vector3Df(0,0,0);
        xidppp = Vector3Df(0,0,0);
        break;
    }
}

void ctrl1::force_reference(Vector3Df &fd, float tactual){
    
    switch(force_behavior->CurrentIndex()){
    case 0:
        // regulation
        fd = Vector3Df(fxd->Value(),fyd->Value(),fzd->Value());
        break;
    
    case 1:
        // tracking
        switch(fx_behavior->CurrentIndex()){
        case 0:
            // regulation
            fd.x = fxd->Value();
            break;
        case 1:
            // sin
            fd.x = afx->Value()*sin(wfx->Value()*tactual)+bfx->Value();
            break;
        case 2:
            // cos
            fd.x = afx->Value()*cos(wfx->Value()*tactual)+bfx->Value();
            break;
        }

        switch(fy_behavior->CurrentIndex()){
        case 0:
            // regulation
            fd.y = fyd->Value();
            break;
        case 1:
            // sin
            fd.y = afy->Value()*sin(wfy->Value()*tactual)+bfy->Value();
            break;
        case 2:
            // cos
            fd.y = afy->Value()*cos(wfy->Value()*tactual)+bfy->Value();
            break;
        }

        switch(fz_behavior->CurrentIndex()){
        case 0:
            // regulation
            fd.z = fzd->Value();
            break;
        case 1:
            // sin
            fd.z = afz->Value()*sin(wfz->Value()*tactual)+bfz->Value();
            break;
        case 2:
            // cos
            fd.z = afz->Value()*cos(wfz->Value()*tactual)+bfz->Value();
            break;
        }
        break;
    
    case 2:
        // trajectory
        break;
    default:
        fd = Vector3Df(0,0,0);
        break;
    }
}


void ctrl1::sliding_ctrl(Euler &torques){
    //flair::core::Time ti = GetTime();
    const AhrsData *refOrientation = GetDefaultReferenceOrientation();
    Quaternion refQuaternion;
    Vector3Df refAngularRates;
    refOrientation->GetQuaternionAndAngularRates(refQuaternion, refAngularRates);
    //flair::core::Time  tf = GetTime()-ti;

    //Printf("ref: %f ms\n", (float)tf/1000000);

    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Quaternion uav_quat;

    

    //ti = GetTime();
    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    //tf = tf = GetTime()-ti;

    //Printf("cur: %f ms\n",  (float)tf/1000000);
    
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    
    float refAltitude, refVerticalVelocity;
    GetDefaultReferenceAltitude(refAltitude, refVerticalVelocity);
    
    float z, zp;
    
    AltitudeValues(z,zp);
    
    float ze = z - refAltitude;
    
    u_sliding->SetValues(ze,zp,currentAngularRates,refAngularRates,currentQuaternion,refQuaternion);
    
    u_sliding->Update(GetTime());
    
    //Thread::Info("%f\t %f\t %f\t %f\n",u_sliding->Output(0),u_sliding->Output(1), u_sliding->Output(2), u_sliding->Output(3));
    
    torques.roll = u_sliding->Output(0);
    torques.pitch = u_sliding->Output(1);
    torques.yaw = u_sliding->Output(2);
    //thrust = u_sliding->Output(3);
    thrust = ComputeDefaultThrust();
    

}

void ctrl1::sliding_ctrl_pos(Euler &torques){
    float tactual=double(GetTime())/1000000000-u_sliding_pos->t0;
    //printf("t: %f\n",tactual);
    Vector3Df xid, xidp, xidpp, xidppp;

    Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
    Quaternion uav_quat;

    //flair::core::Time ti = GetTime();
    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    uavVrpn->GetQuaternion(uav_quat);
    //flair::core::Time  tf = GetTime()-ti;

    //Printf("pos: %f ms\n",  (float)tf/1000000);

    //Thread::Info("Pos: %f\t %f\t %f\n",uav_pos.x,uav_pos.y, uav_pos.z);
    //Printf("Pos: %f\t %f\t %f\n",uav_pos.x,uav_pos.y, uav_pos.z);
    //Printf("Vel: %f\t %f\t %f\n",uav_vel.x,uav_vel.y, uav_vel.z);
    //Thread::Info("Vel: %f\t %f\t %f\n",uav_vel.x,uav_vel.y, uav_vel.z);

    //ti = GetTime();
    const AhrsData *currentOrientation = GetDefaultOrientation();
    Quaternion currentQuaternion;
    Vector3Df currentAngularRates;
    currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
    //tf = GetTime()-ti;

    //Printf("ori: %f ms\n",  (float)tf/1000000);
    
    Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    

    pos_reference(xid, xidp, xidpp, xidppp, tactual);

    //printf("xid: %f\t %f\t %f\n",xid.x,xid.y, xid.z);
    
    u_sliding_pos->SetValues(uav_pos-xid,uav_vel-xidp,xid,xidpp,xidppp,currentAngularRates,currentQuaternion);
    
    u_sliding_pos->Update(GetTime());
    
    //Thread::Info("%f\t %f\t %f\t %f\n",u_sliding->Output(0),u_sliding->Output(1), u_sliding->Output(2), u_sliding->Output(3));
    

    torques.roll = u_sliding_pos->Output(0);
    torques.pitch = u_sliding_pos->Output(1);
    torques.yaw = u_sliding_pos->Output(2);
    thrust = u_sliding_pos->Output(3);
    


}

// void ctrl1::sliding_ctrl_force(Euler &torques){
//     float tactual=double(GetTime())/1000000000-u_sliding_force->t0;
//     //printf("t: %f\n",tactual);
//     Vector3Df xid(0,0,-1), xidp, xidpp, xidppp;

//     Vector3Df uav_pos,uav_vel; // in VRPN coordinate system
//     Quaternion uav_quat;

//     flair::core::Time ti = GetTime();
//     uavVrpn->GetPosition(uav_pos);
//     uavVrpn->GetSpeed(uav_vel);
//     uavVrpn->GetQuaternion(uav_quat);
//     flair::core::Time  tf = GetTime()-ti;

//     //Printf("pos: %f ms\n",  (float)tf/1000000);

//     //Thread::Info("Pos: %f\t %f\t %f\n",uav_pos.x,uav_pos.y, uav_pos.z);
//     //Printf("Pos: %f\t %f\t %f\n",uav_pos.x,uav_pos.y, uav_pos.z);
//     //Printf("Vel: %f\t %f\t %f\n",uav_vel.x,uav_vel.y, uav_vel.z);
//     //Thread::Info("Vel: %f\t %f\t %f\n",uav_vel.x,uav_vel.y, uav_vel.z);

//     ti = GetTime();
//     const AhrsData *currentOrientation = GetDefaultOrientation();
//     Quaternion currentQuaternion;
//     Vector3Df currentAngularRates;
//     currentOrientation->GetQuaternionAndAngularRates(currentQuaternion, currentAngularRates);
//     tf = GetTime()-ti;

//     //Printf("ori: %f ms\n",  (float)tf/1000000);
    
//     Vector3Df currentAngularSpeed = GetCurrentAngularSpeed();
    

//     //printf("xid: %f\t %f\t %f\n",xid.x,xid.y, xid.z);

//     //Vector3Df F = jr3->GetForce();

//     Vector3Df Fd;

//     force_reference(Fd, tactual);

//     pos_reference(xid, xidp, xidpp, xidppp, tactual);
    
//     u_sliding_force->SetValues(uav_pos-xid,uav_vel-xidp,xid,xidpp,xidppp,currentAngularSpeed,currentQuaternion,F,Fd);
    
//     u_sliding_force->Update(GetTime());
    
//     //Thread::Info("%f\t %f\t %f\t %f\n",u_sliding->Output(0),u_sliding->Output(1), u_sliding->Output(2), u_sliding->Output(3));
    
//     torques.roll = u_sliding_force->Output(0);
//     torques.pitch = u_sliding_force->Output(1);
//     torques.yaw = u_sliding_force->Output(2);
//     thrust = u_sliding_force->Output(3);
    
//     //thrust = ComputeDefaultThrust();
// }
