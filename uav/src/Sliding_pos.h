// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file Linear_impl.h
 * \brief Classe permettant le calcul d'un Pid
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2011/05/01
 * \version 4.0
 */

#ifndef SLIDING_POS_H
#define SLIDING_POS_H

#include <Object.h>
#include <ControlLaw.h>
#include <Vector3D.h>

namespace flair {
    namespace core {
        class Matrix;
        class io_data;
    }
    namespace gui {
        class LayoutPosition;
        class DoubleSpinBox;
    }
}

/*! \class Sliding_pos
* \brief Class defining a Sliding_pos
*/

    
    
namespace flair {
    namespace filter {
    /*! \class Sliding_pos
    *
    * \brief Class defining a Sliding_pos
    */
        class Sliding_pos : public ControlLaw {
    
    
public:
    Sliding_pos(const flair::gui::LayoutPosition *position, std::string name);
    ~Sliding_pos();
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
                    flair::core::Vector3Df xidpp, flair::core::Vector3Df xidppp, flair::core::Vector3Df w, flair::core::Quaternion q);
    
    void UseDefaultPlot(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot2(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot3(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot4(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot5(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot6(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot7(const flair::gui::LayoutPosition *position);

    
    
    float t0;

private:
    flair::core::Matrix *state;

    float sech(float value);

    flair::gui::DoubleSpinBox *T, *gamma, *gammap, *alpha, *alphap, *k, *Kd, *Kp, *sat_r, *sat_p, *sat_y, *sat_t, *m, *g, *km, *p, *km_z;
    
    float Sat(float value, float borne);
    
    float delta_t;
    
    bool first_update;
    
    flair::core::Vector3Df sgnp, sgn, sgnp2, sgn2;
    
    
};
} // end namespace filter
} // end namespace flair

#endif // LINEAR_H
