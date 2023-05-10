// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file MetaJR3_impl.h
 * \brief Classe intégrant la manette DualShock3 et les consignes joystick
 * \author Guillaume Sanahuja, Copyright Heudiasyc UMR UTC/CNRS 7253
 * \date 2014/01/14
 * \version 3.4
 */

#ifndef METAJR3_IMPL_H
#define METAJR3_IMPL_H

#include <MetaJR3.h>

namespace flair {
namespace meta {
class MetaJR3;
}
}

/*! \class MetaJR3_impl
*
* \brief Classe intégrant la manette DualShock3 et les consignes joystick
*/
class MetaJR3_impl {
public:
  MetaJR3_impl(flair::meta::MetaJR3 *self, std::string name);
  ~MetaJR3_impl();
  void UpdateFrom(const flair::core::io_data *data);

private:
  flair::meta::MetaJR3 *self;
  flair::core::Matrix *data_s;
};

#endif // METAJR3_IMPL_H