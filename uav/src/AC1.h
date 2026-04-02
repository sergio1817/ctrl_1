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

    /*!
     * \brief Analytical time derivative of the Actor output Ŷr = Ŵa^T σa.
     *
     * Computes  Ẏ̂r = Ẇa^T σa + Ŵa^T σ̇a   exactly, using:
     *   Ẇa  from the adaptation law (3.81),
     *   σ̇a  = diag(σa⊙(1−σa)) Va^T [0; Sr].
     *
     * Must be called AFTER Update() so that the internal variables
     * (W_a, sigma_Va_last_, Wap_last_, Sr_last_) are current.
     */
    Eigen::Vector3f NNaDot() const;
    


private:
    flair::core::Matrix *state;
    bool first_update;
    std::string name;

    flair::gui::DoubleSpinBox *gamma;
    flair::gui::DoubleSpinBox *kw;
    flair::gui::DoubleSpinBox *k;
    
    void updateActor(const Eigen::Vector3f& Sr);
    void computeReward1(const Eigen::Vector3f& e, const Eigen::Vector3f& ep);
    void computeTD(const float& NNc);
    void updateCritic(const Eigen::Vector3f& e);
    
    // Sigmoid function: s = (1-exp(-x))./(1+exp(-x))
    template<typename Derived>
    Eigen::Array<typename Derived::Scalar, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>
    sigmoid1(const Eigen::MatrixBase<Derived>& x) {
        return (1.0 - (-x.array()).exp()) / (1.0 + (-x.array()).exp());
    }

    float sigmoid11(const float& x) {
        return (1.0 - std::exp(-x)) / (1.0 + std::exp(-x));
    }

    bool saturate(Eigen::Vector3f& vec, Eigen::Vector3f min_val, Eigen::Vector3f max_val) {
        bool val = false;
        for (int i = 0; i < vec.size(); ++i) {
            if (vec(i) > max_val(i)) {
                vec(i) = max_val(i);
                val = true;
                //Reset();
            } if (vec(i) < min_val(i)) {
                vec(i) = min_val(i);
                val = true;
                //Reset();
            }
        }
        return val;
    }

    void antiWindup(const Eigen::Vector3f& e);
    
    Eigen::Matrix<float, 4, 10> V_a;
    Eigen::Matrix<float, 4, 10> V_c;

    Eigen::Matrix<float, 10, 3> W_a;
    Eigen::Matrix<float, 10, 1> W_c;

    Eigen::Vector3f int_s = Eigen::Vector3f::Zero();
    //Eigen::Vector3f int_s2 = Eigen::Vector3f::Zero();

    float reward = 0.0F;
    float gamma_val = 0.0F;

    float reward_int = 0.0F;
    float NNc_int = 0.0F;

    double delta_t = 0.0F;

    float NNc = 0.0F;

    Eigen::Vector3f NNa = Eigen::Vector3f::Zero();

    /* Cached intermediates for analytical Ẏr computation */
    Eigen::Matrix<float, 10, 1> sigma_Va_last_;   ///< σa(Va^T χa) from last updateActor
    Eigen::Matrix<float, 10, 3> Wap_last_;         ///< Ẇa from last updateActor
    Eigen::Vector3f Sr_last_;                       ///< Sr from last updateActor
    
};
} // end namespace filter
} // end namespace flair

#endif // AC1_H
