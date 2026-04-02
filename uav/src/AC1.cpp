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
//#include <iostream>

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

    
    //W_c = Eigen::Matrix<float, 1, 10>::Random(1,10);

    GroupBox *critic = new GroupBox(position->NewRow(), "Critic");
    GroupBox *actor = new GroupBox(position->LastRowLastCol(), "Actor");

    gamma = new DoubleSpinBox(actor->NewRow(), "Gamma", 0, 1000, 0.1, 1);
    kw = new DoubleSpinBox(critic->NewRow(), "Kw", 0, 1000, 0.1, 1);
    k = new DoubleSpinBox(critic->LastRowLastCol(), "K", 0, 1000, 0.1, 1);

    AddDataToLog(state);

    V_a << 0.6557F, 0.8235F, 0.2760F, 0.9593F, 0.3517F, 0.1299F, 0.4505F, 0.8687F, 0.8530F, 0.4893F,
            0.0357F, 0.6948F, 0.6797F, 0.5472F, 0.8308F, 0.5688F, 0.0838F, 0.0844F, 0.6221F, 0.3377F,
            0.8491F, 0.3171F, 0.6551F, 0.1386F, 0.5853F, 0.4694F, 0.2290F, 0.3998F, 0.3510F, 0.9001F,
            0.45F,   0.734F,  0.187F,  0.93F,   0.89F,   0.827F,  0.632F,  0.934F,  0.327F,  0.194F;

    V_c << 0.8909F, 0.5472F, 0.1493F, 0.8407F, 0.8143F, 0.9293F, 0.1966F, 0.6160F, 0.3517F, 0.5853F,
            -0.45F,  -0.65F,  -0.154F, -0.953F, -0.343F, -0.794F, -0.154F, -0.934F, -0.315F, -0.765F,
            0.56F,   0.32F,   0.924F,  0.185F,  0.734F,  0.564F,  0.194F,  0.285F,  0.624F,  0.935F,
            0.45F,   0.67F,   0.34F,   0.83F,   0.95F,   0.12F,   0.423F,  0.52F,   0.17F,   0.47F;

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
    //W_a = W_a * 0.001F; // Scale down the initial weights for better learning stability

    W_c << 0.1F, 0.23F, 0.54F, 0.98F, 0.464F, 0.176F, 0.584F, 0.045F, 1.0F, 0.2F;

    sigma_Va_last_.setZero();
    Wap_last_.setZero();
    Sr_last_.setZero();
}

AC1::~AC1() {}


void AC1::UseDefaultPlot(const LayoutPosition *position) {
    DataPlot1D *NNa = new DataPlot1D(position, "NNa_"+this->name, -5, 5);
    NNa->AddCurve(state->Element(0, 0),DataPlot::Red);
    NNa->AddCurve(state->Element(1, 0),DataPlot::Green);
    NNa->AddCurve(state->Element(2, 0),DataPlot::Blue);
}

void AC1::UseDefaultPlot2(const LayoutPosition *position) {
    DataPlot1D *NNc = new DataPlot1D(position, "NNc_"+this->name, -3, 3);
    NNc->AddCurve(state->Element(0, 1),DataPlot::Red);
    NNc->AddCurve(state->Element(1, 1),DataPlot::Green);
    NNc->AddCurve(state->Element(2, 1),DataPlot::Blue);
}


void AC1::Reset() {
    first_update = true;

    reward = 0.0F;
    gamma_val = 0.0F;
    reward_int = 0.0F;
    NNc_int = 0.0F;

    int_s = Eigen::Vector3f::Zero();
    //int_s2 = Eigen::Vector3f::Zero();

    NNa = Eigen::Vector3f::Zero();
    NNc = 0.0F;

    sigma_Va_last_.setZero();
    Wap_last_.setZero();
    Sr_last_.setZero();

    V_a << 0.6557F, 0.8235F, 0.2760F, 0.9593F, 0.3517F, 0.1299F, 0.4505F, 0.8687F, 0.8530F, 0.4893F,
            0.0357F, 0.6948F, 0.6797F, 0.5472F, 0.8308F, 0.5688F, 0.0838F, 0.0844F, 0.6221F, 0.3377F,
            0.8491F, 0.3171F, 0.6551F, 0.1386F, 0.5853F, 0.4694F, 0.2290F, 0.3998F, 0.3510F, 0.9001F,
            0.45F,   0.734F,  0.187F,  0.93F,   0.89F,   0.827F,  0.632F,  0.934F,  0.327F,  0.194F;

    //V_a = V_a * 10.0F; // Scale up the initial weights for better learning signal

    V_c << 0.8909F, 0.5472F, 0.1493F, 0.8407F, 0.8143F, 0.9293F, 0.1966F, 0.6160F, 0.3517F, 0.5853F,
            -0.45F,  -0.65F,  -0.154F, -0.953F, -0.343F, -0.794F, -0.154F, -0.934F, -0.315F, -0.765F,
            0.56F,   0.32F,   0.924F,  0.185F,  0.734F,  0.564F,  0.194F,  0.285F,  0.624F,  0.935F,
            0.45F,   0.67F,   0.34F,   0.83F,   0.95F,   0.12F,   0.423F,  0.52F,   0.17F,   0.47F;

    //V_c = V_c * 10.0F; 

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
    W_a = W_a * 0.001F; // Scale down the initial weights for better learning stability

    W_c << 0.1F, 0.23F, 0.54F, 0.98F, 0.464F, 0.176F, 0.584F, 0.045F, 1.0F, 0.2F;

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

    this->delta_t = (double)(data->DataDeltaTime()) / 1000000000.0F;

    if (first_update) {
        delta_t = 0.0F;
        first_update = false;
    }

    
    computeReward1(e, ep);
    computeTD(NNc);
    updateCritic(e);
    updateActor(Sr);
    //antiWindup(e);

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


void AC1::updateActor(const Eigen::Vector3f& Sr) {
    // Implementation of the actor update logic goes here{

    Eigen::Vector4f chi_a;

    
    int_s = rk4_const(int_s, delta_t, Sr);

    chi_a << 1, int_s;

    Eigen::Matrix<float, 10, 1> sigmoid_Va = sigmoid1(V_a.transpose() * chi_a).matrix();
    if (!sigmoid_Va.allFinite()) {
        return;
    }

    const float gamma_val_local = gamma->Value();
    const float gr = gamma_val * reward;
    const float gr2 = gr * gr;

    /* Forward Euler for W_a update (matches original and Proposition 3.4).
     * Implicit midpoint was tested but alters the effective decay rate
     * of the RL adaptation law, causing drift in regulation. */
    Eigen::Matrix<float, 10, 3> Wap = -gamma_val_local * (sigmoid_Va * Sr.transpose())
                                     - gamma_val_local * W_a * gr2;

    if (!Wap.allFinite()) {
        return;
    }

    Eigen::Matrix<float, 10, 3> W_a_next = rk4_const(W_a, delta_t, Wap);

    if (!W_a_next.allFinite()) {
        return;
    }

    Eigen::Vector3f NNa1 = W_a_next.transpose() * sigmoid_Va;

    if (!NNa1.allFinite()) {
        return;
    }

    /* Cache intermediates for analytical Ẏr computation */
    Wap_last_ = Wap;
    sigma_Va_last_ = sigmoid_Va;
    Sr_last_ = Sr;

    W_a = W_a_next;
    this->NNa = NNa1;
}

void AC1::computeReward1(const Eigen::Vector3f& e, const Eigen::Vector3f& ep) {
    // Implementation of the reward computation logic goes here

    static const Eigen::Matrix3f Q = Eigen::Vector3f(0.9F,0.9F,0.9F).asDiagonal();
    static const Eigen::Matrix3f R = Eigen::Vector3f(0.1F,0.1F,0.1F).asDiagonal();

    reward = 0.5F * (e.transpose() * Q * e + ep.transpose() * R * ep)(0,0);
}

void AC1::computeTD(const float& NNc) {
    // Implementation of the TD computation logic goes here

    float psi = 1000.0F;

    /* Use plain forward Euler for reward_int and NNc_int (matches original).
     * Kahan compensation was tested but the extra precision alters the
     * steady-state equilibrium of the RL temporal difference scheme. */
    reward_int = rk4_const(reward_int, delta_t, reward);
    NNc_int = rk4_const(NNc_int, delta_t, NNc);

    gamma_val = NNc + ((1/psi)*NNc_int) + reward_int;
    
}

void AC1::updateCritic(const Eigen::Vector3f& e) {
    // Implementation of the critic update logic goes here
    float kw_val = kw->Value();
    float K_val = k->Value();
    const float denom_eps = 1.0e-6F;

    Eigen::Vector4f chi_c;
    chi_c << -1, e;

    Eigen::Matrix<float, 10, 1> sigmoid_Va = sigmoid1(V_c.transpose() * chi_c).matrix();
    if (!sigmoid_Va.allFinite()) {
        return;
    }

    const float denom = (sigmoid_Va.transpose() * sigmoid_Va)(0,0);
    if (!std::isfinite(denom) || denom < denom_eps) {
        return;
    }

    const float inv_denom = 1.0F / denom;
    const float sig_gamma = sigmoid11(gamma_val);
    const double dt_local = delta_t;

    /* Forward Euler for W_c update (matches original and Proposition 3.3). */
    Eigen::Matrix<float, 10, 1> sigmoid_Wc = sigmoid1(W_c).matrix();
    if (!sigmoid_Wc.allFinite()) {
        return;
    }

    Eigen::Matrix<float, 10, 1> Wcp = -kw_val * sigmoid_Wc
                                     - K_val * sig_gamma * (sigmoid_Va * inv_denom);
    if (!Wcp.allFinite()) {
        return;
    }

    Eigen::Matrix<float, 10, 1> W_c_next = rk4_const(W_c, delta_t, Wcp);

    if (!W_c_next.allFinite()) {
        return;
    }

    float NNc1 = (W_c_next.transpose() * sigmoid_Va)(0,0);
    if (!std::isfinite(NNc1)) {
        return;
    }

    W_c = W_c_next;
    this->NNc = NNc1;

}


void AC1::antiWindup(const Eigen::Vector3f& e) {
    // Implementation of the anti-windup logic goes here
    Eigen::Vector3f min_val = Eigen::Vector3f(  -2.0F, -2.0F, -6.0F);
    Eigen::Vector3f max_val = Eigen::Vector3f(2.0F, 2.0F, 6.0F);


    if (e.norm()>2.0F) {
        Reset();
        return;
    }

}


Eigen::Vector3f AC1::NNaDot() const {
    /*  Ẏ̂r = Ẇa^T σa  +  Ŵa^T σ̇a
     *
     *  Term 1: Ẇa^T σa
     *    Ẇa (10x3) is cached as Wap_last_
     *    σa  (10x1) is cached as sigma_Va_last_
     *    → Ẇa^T σa = (10x3)^T (10x1) = (3x1)
     *
     *  Term 2: Ŵa^T σ̇a
     *    σ̇a = diag(σa ⊙ (1−σa)) · Va^T · χ̇a
     *    where χ̇a = [0; Sr]  (4x1)
     *    σa ⊙ (1−σa) is the sigmoid derivative  (element-wise, 10x1)
     *    Va^T (10x4) · χ̇a (4x1) = (10x1)
     *    → σ̇a = (σa ⊙ (1−σa)) ⊙ (Va^T χ̇a)   (10x1, element-wise)
     *    → Ŵa^T σ̇a = (10x3)^T (10x1) = (3x1)
     */

    /* Term 1: Ẇa^T σa */
    Eigen::Vector3f term1 = Wap_last_.transpose() * sigma_Va_last_;

    /* Term 2: Ŵa^T σ̇a */
    /* χ̇a = [0; Sr_last_] */
    Eigen::Vector4f chi_a_dot;
    chi_a_dot << 0.0F, Sr_last_;

    /* Va^T χ̇a  (10x1) */
    Eigen::Matrix<float, 10, 1> Va_T_chi_dot = V_a.transpose() * chi_a_dot;

    /* σ'(x) = σ(x) ⊙ (1 − σ(x))  for the sigmoid s=(1-exp(-x))/(1+exp(-x))
     * Note: for this sigmoid, σ'(x) = (1 − σ²(x)) / 2
     * since σ(x) = tanh(x/2)*... actually let's be precise.
     * sigmoid1(x) = (1-exp(-x))/(1+exp(-x)) = tanh(x/2)
     * d/dx tanh(x/2) = (1/2) sech²(x/2) = (1/2)(1 - tanh²(x/2))
     *                = (1 - σ²) / 2
     */
    Eigen::Array<float, 10, 1> sigma_arr = sigma_Va_last_.array();
    Eigen::Array<float, 10, 1> sigma_deriv = 0.5F * (1.0F - sigma_arr * sigma_arr);

    /* σ̇a = sigma_deriv ⊙ (Va^T χ̇a) */
    Eigen::Matrix<float, 10, 1> sigma_dot = (sigma_deriv * Va_T_chi_dot.array()).matrix();

    /* Ŵa^T σ̇a */
    Eigen::Vector3f term2 = W_a.transpose() * sigma_dot;

    return term1 + term2;
}

} // end namespace filter
} // end namespace flair
