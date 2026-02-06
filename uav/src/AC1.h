// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file AC1.h
 * \brief 
 * \author Sergio Urzua, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2026/02/02
 * \version 1.0
 */

#ifndef AC1_H
#define AC1_H

#include <Object.h>
#include <ControlLaw.h>
#include <Vector3D.h>
#include <Eigen/Core>
#include "NMethods.h"

namespace flair {
    namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class GroupBox;
        class DoubleSpinBox;
        class CheckBox;
        class Label;
    }
}

/*! \class AC1
* \brief Class defining a AC1
*/

    
    
namespace flair {
    namespace filter {
    /*! \class AC1
    *
    * \brief Class defining a AC1
    */
        class AC1 : public ControlLaw {
    

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    AC1(const flair::gui::GroupBox *position, std::string name);
    ~AC1();
    void UpdateFrom(const flair::core::io_data *data) override;
    void SetValues(const Eigen::Vector3f& e, const Eigen::Vector3f& ep, const Eigen::Vector3f& Sr);
    void Reset() override;
    void UseDefaultPlot(const flair::gui::LayoutPosition *position) override;
    void UseDefaultPlot2(const flair::gui::LayoutPosition *position);
    


private:
    flair::core::Matrix *state;
    bool first_update;
    std::string name;

    flair::gui::DoubleSpinBox *gamma;
    flair::gui::DoubleSpinBox *kw;
    flair::gui::DoubleSpinBox *k;
    
    void updateActor(Eigen::Vector3f Sr);
    void computeReward1(Eigen::Vector3f e, Eigen::Vector3f ep);
    void computeTD(float NNc);
    void updateCritic(Eigen::Vector3f e);
    
    // Sigmoid function: s = (1-exp(-x))./(1+exp(-x))
    template<typename Derived>
    Eigen::Array<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
    sigmoid1(const Eigen::MatrixBase<Derived>& x) {
        return (1.0 - (-x.array()).exp()) / (1.0 + (-x.array()).exp());
    }

    float sigmoid11(const float& x) {
        return (1.0 - std::exp(-x)) / (1.0 + std::exp(-x));
    }

    bool saturate(Eigen::Vector3f& vec, float min_val, float max_val) {
        for (int i = 0; i < vec.size(); ++i) {
            if (vec(i) > max_val) {
                vec(i) = max_val;
                return true;
                //Reset();
            } if (vec(i) < min_val ) {
                vec(i) = min_val;
                return true;
                //Reset();
            }
        }
        return false;
    }

    void antiWindup(const Eigen::Vector3f& e);
    
    Eigen::Matrix<float, 4, 10> V_a;
    Eigen::Matrix<float, 4, 10> V_c;

    Eigen::Matrix<float, 10, 3> W_a;
    Eigen::Matrix<float, 10, 1> W_c;

    Eigen::Vector3f int_s = Eigen::Vector3f::Zero();
    //Eigen::Vector3f int_s2 = Eigen::Vector3f::Zero();

    float reward = 0.0F;
    float reward_int = 0.0F;
    float gamma_val = 0.0F;

    float NNc_int = 0.0F;

    float delta_t = 0.0F;

    float NNc = 0.0F;

    Eigen::Vector3f NNa = Eigen::Vector3f::Zero();
    
    
};
} // end namespace filter
} // end namespace flair

#endif // AC1_H
