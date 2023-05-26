// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file Slifing_force.h
 * \brief Class defining a Sliding mode controller for position
 * \author Sergio Urzua, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2023/05/25
 * \version 1.0
 */

#ifndef SLIDING_FORCE_H
#define SLIDING_FORCE_H

#include <Object.h>
#include "NMethods.h"
#include <ControlLaw.h>
#include <Eigen/Dense>
#include <Vector3D.h>

namespace flair {
    namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
        class CheckBox;
        class Label;
    }
}

/*! \class Sliding_force
* \brief Class defining a Sliding_force
*/

    
    
namespace flair {
    namespace filter {
    /*! \class Sliding_force
    *
    * \brief Class defining a Sliding_force
    */
        class Sliding_force : public ControlLaw {
    
    
public:
    Sliding_force(const flair::gui::LayoutPosition *position, std::string name);
    ~Sliding_force();
    void UpdateFrom(const flair::core::io_data *data);
    void Reset(void);
    
    /*!
  * \brief Set input values
  *
  * \param xie Error de posicion
  * \param xiep Error de velocidad
  * \param xid  Posicion deseada
  * \param xidpp Aceleracion deseada
  * \param xidppp Jerk deseado
  * \param w Velocidad angular
  * \param q Cuaternio de orientacion
  */
    void SetValues(flair::core::Vector3Df xie, flair::core::Vector3Df xiep, flair::core::Vector3Df xid, 
                    flair::core::Vector3Df xidpp, flair::core::Vector3Df xidppp, flair::core::Vector3Df w, flair::core::Quaternion q,
                    flair::core::Vector3Df F, flair::core::Vector3Df Fd);
    
    void UseDefaultPlot(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot2(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot3(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot4(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot5(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot6(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot7(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot8(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot9(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot10(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot11(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot12(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot13(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot14(const flair::gui::LayoutPosition *position);
    
    
    flair::core::Time t0;
    float tactual;

private:
    flair::core::Matrix *state;
    Levant_diff levant;

    float sech(float value);

    flair::gui::CheckBox *levantd;
    flair::gui::DoubleSpinBox *T, *gamma, *k, *sat_r, *sat_p, *sat_y, *sat_t, *m, *g, *km, *p, *km_z;
    flair::gui::DoubleSpinBox *gamma_roll, *gamma_pitch, *gamma_yaw, *gamma_x, *gamma_y, *gamma_z;
    flair::gui::DoubleSpinBox *alpha_roll, *alpha_pitch, *alpha_yaw, *alpha_x, *alpha_y, *alpha_z;
    flair::gui::DoubleSpinBox *Kd_roll, *Kd_pitch, *Kd_yaw, *Kd_x, *Kd_y, *Kd_z;
    flair::gui::DoubleSpinBox *Kp_roll, *Kp_pitch, *Kp_yaw, *Kp_x, *Kp_y, *Kp_z;
    flair::gui::DoubleSpinBox *alpha_l,*lamb_l;
    flair::gui::DoubleSpinBox *gamma_fx, *gamma_fy, *gamma_fz, *beta_1, *beta_2, *eta, *mu, *kf, *kf_x, *alpha_fx;

    flair::gui::Label *lo, *lp;
    
    float Sat(float value, float borne);
    
    float delta_t, sgnfx, sgnfxp;
    
    bool first_update;
    
    Eigen::Vector3f sgnpos_p, sgnpos, sgnori_p, sgnori;

    Eigen::Vector2f Sf, dLamb;
    Eigen::Vector2f sgnf, sgnfp;

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity(3,3);

    void Position(Eigen::Quaternionf &qd, Eigen::Vector3f &wd, float &Trs, const Eigen::Vector3f xie, const Eigen::Vector3f xiep, const Eigen::Vector3f xid,
                            const Eigen::Vector3f xidpp, const Eigen::Vector3f xidppp, const Eigen::Quaternionf q);
    
    void ForcePosition(Eigen::Quaternionf &qd, Eigen::Vector3f &wd, float &Trs, const Eigen::Vector3f xie, const Eigen::Vector3f xiep, const Eigen::Vector3f xid,
                            const Eigen::Vector3f xidpp, const Eigen::Vector3f xidppp, const Eigen::Quaternionf q, const Eigen::Vector3f F, const Eigen::Vector3f Fd);

    void Orientation(Eigen::Vector3f &tau, const Eigen::Quaternionf qd, const Eigen::Vector3f wd, const Eigen::Quaternionf q, const Eigen::Vector3f w);

    void ForcePitch(float &tau_pitch, const float dFx, const float dPitch, const float dPitchp);

    //flair::core::Vector3ff sgnori_p, sgnori;
    
    
};
} // end namespace filter
} // end namespace flair

#endif // LINEAR_H
