// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2023/05/12
//  filename:   TargetJR3.h
//
//  author:     Sergio Urzua
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    Base class for target side JR3 sensor.
//
//
/*********************************************************************/

#ifndef TargetJR3_H
#define TargetJR3_H

#include <IODevice.h>
#include <Thread.h>
#include <stdint.h>
#include <Vector3D.h>
#include <queue>

namespace flair
{
  namespace core
  {
    class FrameworkManager;
    class Matrix;
    class Socket;
    class io_data;
  }
  namespace gui
  {
    class Tab;
    class TabWidget;
    class DataPlot1D;
    class PushButton;
    class SpinBox;
    class Label;
  }
}

namespace flair
{
  namespace sensor
  {
    enum class ControllerAction;

    /*! \class TargetJR3
     *
     * \brief Base Class for target side JR3 sensor.
     *
     */
    class TargetJR3 : public core::Thread, public core::IODevice
    {
    public:
      TargetJR3(std::string name, uint8_t priority = 0);
      ~TargetJR3();
      // void DrawUserInterface();
      virtual bool IsConnected() const = 0;
      virtual bool IsDataFrameReady() = 0;
      // axis stuff
      unsigned int GetAxisNumber() const;
      virtual std::string GetAxisName(unsigned int axisId) const;
      float
      GetAxisValue(unsigned int axisId) const;
      // controller state stuff

      void UpdateFrom(const core::io_data *data){}; // TODO
      void DrawUserInterface();
      gui::Tab *GetTab(void) const;
      flair::gui::TabWidget *tab;
      flair::gui::PushButton *setTs;
      flair::gui::SpinBox *T;
      flair::gui::Label *l2;

      /*!
       * \brief Get force x axis
       *
       * \return Fx
       */
      float GetFx() const;

      /*!
       * \brief Get force y axis
       *
       * \return Fy
       */
      float GetFy() const;

      /*!
       * \brief Get force z axis
       *
       * \return Fz
       */
      float GetFz() const;

      /*!
       * \brief Get moment x axis
       *
       * \return Mx
       */
      float GetMx() const;

      /*!
       * \brief Get moment y axis
       *
       * \return My
       */
      float GetMy() const;

      /*!
       * \brief Get moment z axis
       *
       * \return Mz
       */
      float GetMz() const;

      /*!
       * \brief Get force vector
       *
       * \return Force vector
       */
      flair::core::Vector3Df GetForce() const;

      /*!
       * \brief Get moment vector
       *
       * \return Moment vector
       */
      flair::core::Vector3Df GetMoment() const;

      core::Matrix *axis = nullptr;

      flair::core::Time dt_read;

    protected:
      virtual bool ProcessMessage(core::Message *msg) = 0;
      void QueueMessage(core::Message msg);
      virtual bool SensorInitialization() = 0; // {return true;};
      // axis stuff
      unsigned int axisNumber;
      gui::DataPlot1D **axisPlot;

      virtual void AcquireSensorData(core::Matrix &axis) = 0; // responsible for
                                                              // getting the axis
                                                              // data from the
                                                              // hardware
      uint16_t bitsPerAxis;
      // button stuff
      unsigned int buttonNumber;
      // controller state stuff
      unsigned int ledNumber;

    private:
      void Run();

      std::queue<core::Message *> changeStateQueue;
      flair::gui::Tab *main_tab;
      flair::gui::Tab *setup_tab;
    };
  }
}

#endif // TargetJR3_H
