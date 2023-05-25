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

/*! \class Linear
* \brief Class defining a Linear
*/

    
    
namespace flair {
    namespace filter {
    /*! \class Linear
    *
    * \brief Class defining a Linear
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
  * \param we Error en w
  * \param q  Cuaternio actual
  * \param qd Cuaternio deseado
  */
    void SetValues(float ze, float zp, flair::core::Vector3Df we, flair::core::Quaternion q, flair::core::Quaternion qd);
    
    void UseDefaultPlot(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot2(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot3(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot4(const flair::gui::LayoutPosition *position);
    void UseDefaultPlot5(const flair::gui::LayoutPosition *position);
    
    flair::core::Time t0;

private:
    flair::core::Matrix *state;

    flair::gui::DoubleSpinBox *T, *k1, *k2, *gamma, *alpha, *k, *Kd, *sat_r, *sat_p, *sat_y, *sat_t, *m, *g, *km, *p;
    
    float Sat(float value, float borne);
    
    float delta_t;
    
    bool first_update;
    
    flair::core::Vector3Df sgnp, sgn;
    
    
};
} // end namespace filter
} // end namespace flair

#endif // LINEAR_H
