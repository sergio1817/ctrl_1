/*!
 * \file NMethods.h
 * \brief Numerical Methods Library
 * \author Sergio Urzua, et al. (alumni of RYMA)
 * \date 2023/05/18
 * \version 1.0
 */

#ifndef NMETHODS_H
#define NMETHODS_H

#include <Eigen/Dense>


/*!
  * \brief 4 order Runge-Kutta integration method.
  *
  * Compute the integral of a function using the Runge-Kutta method of 4th order.
  *
  * \param fPtr Pointer to the function to integrate
  * \param iC   Integral of the function
  * \param iCdt Derivative of the integral of the function
  * \param dt   Time step
  */
float rk4(float(*fPtr)(float), const float iC, const float iCdt, const float dt);

/*!
  * \brief 4 order Runge-Kutta integration method.
  *
  * Compute the integral of a function using the Runge-Kutta method of 4th order for a Eigen::Vector3d object.
  *
  * \param iC   Integral of the function
  * \param iCdt Derivative of the integral of the function
  * \param dt   Time step
  */
Eigen::Vector3f rk4_vec(const Eigen::Vector3f iC, const Eigen::Vector3f iCdt, const float dt);


/*! \fn function1d.
 */
float function1d(float iCdt);

/*! \fn sign, función que contiene la función signo.
 */
float sign(const float a);

float sigmoide(const float a, const float d);

float signth(const float a, const float p);

Eigen::Vector3f signth(const Eigen::Vector3f a, const float p);



class Levant_diff{
public:
    Levant_diff(std::string mode = "tanh", const float aplha = 1, const float lamb = 1, const float p = 3000);
    ~Levant_diff();
    void setParam(const float alpha, const float lamb, const float p);
    void setParam(const float alpha, const float lamb);
    void Reset(void);
    float Compute(const float f, const float dt);
    void Compute(float &u, const float f, const float dt);

    Eigen::Vector3f Compute(const Eigen::Vector3f f, const float dt);

private:
    std::string mode;
    float alpha, lamb, p;
    float u, u1, u1p, x;

    Eigen::Vector3f u_vec, u1_vec, u1p_vec, x_vec;

    float sign_(const float a);

};

#endif // NMETHODS_H
