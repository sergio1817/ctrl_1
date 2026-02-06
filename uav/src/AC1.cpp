// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2026/02/02
//  filename:   AC1.cpp
//
//  author:     Sergio Urzua
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    
//
//
/*********************************************************************/
#include "AC1.h"
#include "NMethods.h"
#include <Eigen/src/Core/Array.h>
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
#include <iostream>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

namespace flair {
namespace filter {

AC1::AC1(const GroupBox *position, string name): ControlLaw(position, name, 3), first_update(true), name(name){
    input = new Matrix(this, 3, 3, floatType, name);


    MatrixDescriptor *desc = new MatrixDescriptor(3, 2);
    desc->SetElementName(0, 0, "NNa_1");
    desc->SetElementName(1, 0, "NNa_2");
    desc->SetElementName(2, 0, "NNa_3");
    desc->SetElementName(0, 1, "NNc");
    desc->SetElementName(1, 1, "reward");
    desc->SetElementName(2, 1, "gamma_val");
    state = new Matrix(this, desc, floatType, name);
    delete desc;
    
    V_a = Eigen::Matrix<float, 4, 10>::Random(4,10);
    V_c = Eigen::Matrix<float, 4, 10>::Random(4,10);

    W_a = Eigen::Matrix<float, 10, 3>::Random(10,3)*0.001F;
    W_c = Eigen::Matrix<float, 1, 10>::Random(1,10);

    GroupBox *critic = new GroupBox(position->NewRow(), "Critic");
    GroupBox *actor = new GroupBox(position->LastRowLastCol(), "Actor");

    gamma = new DoubleSpinBox(actor->NewRow(), "Gamma", 0, 1000, 0.1, 1);
    kw = new DoubleSpinBox(critic->NewRow(), "Kw", 0, 1000, 0.1, 1);
    k = new DoubleSpinBox(critic->LastRowLastCol(), "K", 0, 1000, 0.1, 1);
    
    
}

AC1::~AC1() {}


void AC1::UseDefaultPlot(const LayoutPosition *position) {
    DataPlot1D *NNa = new DataPlot1D(position, "NNa_"+this->name, -5, 5);
    NNa->AddCurve(state->Element(0, 0),DataPlot::Red);
    NNa->AddCurve(state->Element(1, 0),DataPlot::Green);
    NNa->AddCurve(state->Element(2, 0),DataPlot::Blue);
}

void AC1::UseDefaultPlot2(const LayoutPosition *position) {
    DataPlot1D *NNc = new DataPlot1D(position, "NNc_"+this->name, -5, 5);
    NNc->AddCurve(state->Element(0, 1),DataPlot::Red);
    NNc->AddCurve(state->Element(1, 1),DataPlot::Green);
    NNc->AddCurve(state->Element(2, 1),DataPlot::Blue);
}


void AC1::Reset() {
    first_update = true;


    reward_int = 0.0F;
    NNc_int = 0.0F;

    int_s = Eigen::Vector3f::Zero();
    int_s2 = Eigen::Vector3f::Zero();

    V_a = Eigen::Matrix<float, 4, 10>::Random(4,10);
    V_c = Eigen::Matrix<float, 4, 10>::Random(4,10);

    W_a = Eigen::Matrix<float, 10, 3>::Random(10,3)*0.001F;
    W_c = Eigen::Matrix<float, 1, 10>::Random(1,10);

    NNa = Eigen::Vector3f::Zero();
    NNc = 0.0F;

    reward = 0.0F;
    gamma_val = 0.0F;

}


void AC1::SetValues(const Eigen::Vector3f& e, const Eigen::Vector3f& ep, const Eigen::Vector3f& Sr) {
    input->SetValue(0, 0, e(0));
    input->SetValue(1, 0, e(1));
    input->SetValue(2, 0, e(2));

    input->SetValue(0, 1, ep(0));
    input->SetValue(1, 1, ep(1));
    input->SetValue(2, 1, ep(2));

    input->SetValue(0, 2, Sr(0));
    input->SetValue(1, 2, Sr(1));
    input->SetValue(2, 2, Sr(2));
}


void AC1::UpdateFrom(const io_data *data) {
    //Implementation of the update logic goes here

    

    const Matrix* input = dynamic_cast<const Matrix*>(data);
  
    if (!input) {
        Warn("casting %s to Matrix failed\n",data->ObjectName().c_str(),TIME_INFINITE);
        return;
    }

    input->GetMutex();

    Eigen::Vector3f e(input->ValueNoMutex(0, 0),input->ValueNoMutex(1, 0),input->ValueNoMutex(2, 0));
    Eigen::Vector3f ep(input->ValueNoMutex(0, 1),input->ValueNoMutex(1, 1),input->ValueNoMutex(2, 1));
    Eigen::Vector3f Sr(input->ValueNoMutex(0, 2),input->ValueNoMutex(1, 2),input->ValueNoMutex(2, 2));
    
    input->ReleaseMutex();

    this->delta_t = (float)(data->DataDeltaTime()) / 1000000000.0F;
    computeReward1(e, ep);
    computeTD(NNc);
    updateCritic(e);
    updateActor(Sr);
    antiWindup();

    state->GetMutex();
    state->SetValue(0, 0, NNa(0));
    state->SetValue(1, 0, NNa(1));
    state->SetValue(2, 0, NNa(2));
    state->SetValue(0, 1, NNc);
    state->SetValue(1, 1, reward);
    state->SetValue(2, 1, gamma_val);
    state->ReleaseMutex();
    
    output->SetValue(0, 0, NNa(0));
    output->SetValue(1, 0, NNa(1));
    output->SetValue(2, 0, NNa(2));
    output->SetDataTime(data->DataTime());
    
    ProcessUpdate(output);
}


void AC1::updateActor(Eigen::Vector3f Sr) {
    // Implementation of the actor update logic goes here{

    Eigen::Vector4f chi_a;

    
    int_s = rk4_eigen(int_s, Sr, delta_t);
    std::cout<<"int_s: " << int_s.transpose() << '\n';
    int_s2 = rk4_vec(int_s2, Sr, delta_t);
    std::cout<<"int_s2: " << int_s2.transpose() << '\n';


    chi_a << 1, int_s;

    Eigen::VectorXf sigmoid_Va = sigmoid1(V_a.transpose() * chi_a);

    std::cout<<"sigmoid_Va: " << sigmoid_Va.transpose() << '\n';

    Eigen::Matrix<float, 10, 10> Gamma = gamma->Value() * Eigen::Matrix<float,10,10>::Identity();

    Eigen::Matrix<float, 10, 3> Wap = -Gamma*sigmoid_Va*Sr.transpose() - Gamma*W_a*(gamma_val*reward)*(gamma_val*reward);


    W_a = rk4_eigen_matrix(W_a, Wap, delta_t);

    NNa = W_a.transpose()*sigmoid_Va;
}

void AC1::computeReward1(Eigen::Vector3f e, Eigen::Vector3f ep) {
    // Implementation of the reward computation logic goes here

    Eigen::Matrix3f Q = Eigen::Vector3f(0.9,0.9,0.9).asDiagonal();
    Eigen::Matrix3f R = Eigen::Vector3f(0.1,0.1,0.1).asDiagonal();

    reward = 0.5F * (e.transpose() * Q * e + ep.transpose() * R * ep)(0,0);
}

void AC1::computeTD(float NNc) {
    // Implementation of the TD computation logic goes here

    float psi = 1000.0F;

    reward_int = rk4(function1d, reward_int, reward, delta_t);
    NNc_int = rk4(function1d, NNc_int, NNc, delta_t);

    gamma_val = NNc + ((1/psi)*NNc_int) + reward_int;
    
}

void AC1::updateCritic(Eigen::Vector3f e) {
    // Implementation of the critic update logic goes here
    float kw_val = kw->Value();
    float K_val = k->Value();

    Eigen::Vector4f chi_c;
    chi_c << -1, e;

    Eigen::Vector<float, 10> sigmoid_Va = sigmoid1(V_c.transpose() * chi_c);

    Eigen::Matrix<float, 10, 1> Wcp = -kw_val*(sigmoid1(W_c)) - K_val*std::tanh(gamma_val*500)*((sigmoid_Va)/(sigmoid_Va.transpose()*sigmoid_Va))(0,0);

    W_c = rk4_eigen_matrix(W_c,Wcp, delta_t);

    NNc =  (W_c.transpose()*sigmoid_Va)(0,0);

    std::cout<<"NNc: " << NNc << '\n';

}


void AC1::antiWindup() {
    // Implementation of the anti-windup logic goes here
    float min_val = -8.0F;
    float max_val = 8.0F;
    if (saturate(NNa, min_val, max_val)) {
        Reset();
    };

}


} // end namespace filter
} // end namespace flair
