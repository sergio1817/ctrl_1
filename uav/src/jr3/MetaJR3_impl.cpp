// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2014/01/14
//  filename:   MetaJR3_impl.cpp
//
//  author:     Guillaume Sanahuja
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    objet integrant la manette DualShock3 et les consignes joystick
//
//
/*********************************************************************/

#include "MetaJR3_impl.h"
#include "MetaJR3.h"
#include <JoyReference.h>
#include <Tab.h>
#include <Matrix.h>
#include <FrameworkManager.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;
using namespace flair::meta;

MetaJR3_impl::MetaJR3_impl(MetaJR3 *self, string name) {
  this->self = self;

  data_s = new Matrix(self, 6, 1, floatType, name);
}

MetaJR3_impl::~MetaJR3_impl() {}

// receives updates from the controler
void MetaJR3_impl::UpdateFrom(const io_data *data) {
  const Matrix* input = dynamic_cast<const Matrix*>(data);
  
  if (!input) {
      self->Warn("casting %s to Matrix failed\n",data->ObjectName().c_str());
      return;
  }

  // on prend une fois pour toute le mutex et on fait des accès directs
  input->GetMutex();

  

  if (!getFrameworkManager()->ConnectionLost()) {
    input->GetMutex();
    data_s->SetValue(0,0,input->ValueNoMutex(0, 0));
    data_s->SetValue(1,0,input->ValueNoMutex(1, 0));
    data_s->SetValue(2,0,input->ValueNoMutex(2, 0));
    data_s->SetValue(3,0,input->ValueNoMutex(3, 0));
    data_s->SetValue(4,0,input->ValueNoMutex(4, 0));
    data_s->SetValue(5,0,input->ValueNoMutex(5, 0));

    input->ReleaseMutex();
    }else{
    data_s->SetValue(0,0,0);
    data_s->SetValue(1,0,0);
    data_s->SetValue(2,0,0);
    data_s->SetValue(3,0,0);
    data_s->SetValue(4,0,0);
    data_s->SetValue(5,0,0);
  }
  input->ReleaseMutex();


}
}
