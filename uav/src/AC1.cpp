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
    
    //V_a = Eigen::Matrix<float, 4, 10>::Random(4,10);
    //V_c = Eigen::Matrix<float, 4, 10>::Random(4,10)*10.0F;    

    // W_a << 0.15F, 0.3F, 0.5F,
    //         0.34F, 0.85F, 0.67F,
    //         0.27F, 0.83F, 0.87F,
    //         0.97F, 0.47F, 0.62F,
    //         0.74F, 0.38F, 0.31F,
    //         0.93F, 0.16F, 0.24F,
    //         0.26F, 0.27F, 0.72F,
    //         0.05F, 0.42F, 0.69F,
    //         0.49F, 0.75F, 0.43F,
    //         0.83F, 0.92F, 0.98F;

    
    //W_c = Eigen::Matrix<float, 1, 10>::Random(1,10);

    GroupBox *critic = new GroupBox(position->NewRow(), "Critic");
    GroupBox *actor = new GroupBox(position->LastRowLastCol(), "Actor");

    gamma = new DoubleSpinBox(actor->NewRow(), "Gamma", 0, 1000, 0.1, 1);
    kw = new DoubleSpinBox(critic->NewRow(), "Kw", 0, 1000, 0.1, 1);
    k = new DoubleSpinBox(critic->LastRowLastCol(), "K", 0, 1000, 0.1, 1);

    AddDataToLog(state);
    
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

    reward = 0.0F;
    reward_int = 0.0F;
    gamma_val = 0.0F;

    NNc_int = 0.0F;

    int_s = Eigen::Vector3f::Zero();
    //int_s2 = Eigen::Vector3f::Zero();

    V_a << 0.6557F, 0.8235F, 0.2760F, 0.9593F, 0.3517F, 0.1299F, 0.4505F, 0.8687F, 0.8530F, 0.4893F,
            0.0357F, 0.6948F, 0.6797F, 0.5472F, 0.8308F, 0.5688F, 0.0838F, 0.0844F, 0.6221F, 0.3377F,
            0.8491F, 0.3171F, 0.6551F, 0.1386F, 0.5853F, 0.4694F, 0.2290F, 0.3998F, 0.3510F, 0.9001F,
            0.45F,   0.734F,  0.187F,  0.93F,   0.89F,   0.827F,  0.632F,  0.934F,  0.327F,  0.194F;
    V_c << 0.8909F, 0.5472F, 0.1493F, 0.8407F, 0.8143F, 0.9293F, 0.1966F, 0.6160F, 0.3517F, 0.5853F,
            -0.45F,  -0.65F,  -0.154F, -0.953F, -0.343F, -0.794F, -0.154F, -0.934F, -0.315F, -0.765F,
            0.56F,   0.32F,   0.924F,  0.185F,  0.734F,  0.564F,  0.194F,  0.285F,  0.624F,  0.935F,
            0.45F,   0.67F,   0.34F,   0.83F,   0.95F,   0.12F,   0.423F,  0.52F,   0.17F,   0.47F;
    //V_c *= 10.0F;

    W_a << 0.15F, 0.3F, 0.5F,
            0.34F, 0.85F, 0.67F,
            0.27F, 0.83F, 0.87F,
            0.97F, 0.47F, 0.62F,
            0.74F, 0.38F, 0.31F,
            0.93F, 0.16F, 0.24F,
            0.26F, 0.27F, 0.72F,
            0.05F, 0.42F, 0.69F,
            0.49F, 0.75F, 0.43F,
            0.83F, 0.92F, 0.98F;
    W_a *= 0.001F;

    W_c << 0.1F, 0.23F, 0.54F, 0.98F, 0.464F, 0.176F, 0.584F, 0.045F, 1.0F, 0.2F;

    NNa = Eigen::Vector3f::Zero();
    NNc = 0.0F;

    
    

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

    if (first_update) {
        delta_t = 0.0F;
        first_update = false;
    }

    
    computeReward1(e, ep);
    computeTD(NNc);
    updateCritic(e);
    updateActor(Sr);
    antiWindup(e);

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
    //std::cout<<"int_s: " << int_s.transpose() << '\n';
    //int_s2 = rk4_vec(int_s2, Sr, delta_t);
    //std::cout<<"int_s2: " << int_s2.transpose() << '\n';


    chi_a << 1, int_s;

    Eigen::VectorXf sigmoid_Va = sigmoid1(V_a.transpose() * chi_a);

    //std::cout<<"sigmoid_Va: " << sigmoid_Va.transpose() << '\n';

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


void AC1::antiWindup(const Eigen::Vector3f& e) {
    // Implementation of the anti-windup logic goes here
    float min_val = -8.0F;
    float max_val = 8.0F;

    if (NNa.hasNaN() || std::isnan(NNc)) {
        Reset();
        return;
    }

    if (e.norm()>2.0F) {
        Reset();
        return;
    }

    if (saturate(NNa, min_val, max_val) ) {
        Reset();
    };

}


} // end namespace filter
} // end namespace flair
