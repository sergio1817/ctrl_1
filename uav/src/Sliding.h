// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file Sliding.h
 * \brief Classe permettant le calcul d'un Pid
 * \author Sergio Urzua, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef SLIDING_H
#define SLIDING_H

#include <Object.h>
#include <ControlLaw.h>
#include <Vector3D.h>
#include <Eigen/Dense>

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

/*! \class Sliding
* \brief Class defining a Sliding
*/

    
    
namespace flair {
    namespace filter {
    /*! \class Sliding
    *
    * \brief Class defining a Sliding
    */
        class Sliding : public ControlLaw {
    
    
public:
    Sliding(const flair::gui::LayoutPosition *position, std::string name);
    ~Sliding();
    void UpdateFrom(const flair::core::io_data *data);
    void Reset(void);
    
    /*!
  * \brief Set input values
  *
  * \param ze Error en z
  * \param zp Error en zp
  * \param w  Velocidad angular actual
  * \param wd Velocidad angular deseada
  * \param q  Cuaternio actual
  * \param qd Cuaternio deseado
  */
    void SetValues(float ze, float zp, flair::core::Vector3Df w, flair::core::Vector3Df wd, flair::core::Quaternion q, flair::core::Quaternion qd);
    
    void UseDefaultPlot(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot2(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot3(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot4(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot5(const flair::gui::LayoutPosition *position);
    
    flair::core::Time t0;

private:
    flair::core::Matrix *state;

    flair::gui::DoubleSpinBox *T, *k1, *k2, *gamma, *alpha, *k, *Kd, *sat_r, *sat_p, *sat_y, *sat_t, *m, *g, *km, *p;
    flair::gui::DoubleSpinBox *alpha_roll, *alpha_pitch, *alpha_yaw;
    flair::gui::DoubleSpinBox *gamma_roll, *gamma_pitch, *gamma_yaw;
    flair::gui::DoubleSpinBox *Kd_roll, *Kd_pitch, *Kd_yaw;

    flair::gui::Label *lo;
    
    float Sat(float value, float borne);
    
    float delta_t;
    
    bool first_update;
    
    Eigen::Vector3f sgnori_p, sgnori;
    
    
};
} // end namespace filter
} // end namespace flair

#endif // LINEAR_H
