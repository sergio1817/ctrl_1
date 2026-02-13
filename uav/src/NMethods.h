/*!
 * \file NMethods.h
 * \brief Numerical Methods Library
 * \author Sergio Urzua, et al. (alumni of RYMA)
 * \date 2023/05/18
 * \version 1.0
 */

#ifndef NMETHODS_H
#define NMETHODS_H

#include <Eigen/Core>
#include <cmath>
#include <type_traits>
//#include <cstdint>
//#include <utility>
//#include <functional>


/*!
  * \brief 4 order Runge-Kutta integration method.
  *
  * Compute the integral of a function using the Runge-Kutta method of 4th order.
  *
  * \param fPtr Pointer to the function to integrate
  * \param iC   Integral of the function
  * \param iCdt Derivative of the integral of the function
  * \param dt   Time step
  * 
  * \return Integral of the function
  */
float rk4o(float(*fPtr)(float),  float iC, const float iCdt, const float dt);

/*! \fn function1d.
 */
float function1d(float iCdt);

/*!
  * \brief 4th order Runge-Kutta integration for scalar state with non-linear dynamics.
  *
  * \tparam DerivFunc Function type that computes derivative: float f(float x)
  *
  * \param x Current state
  * \param dt Time step
  * \param derivative Function object that computes dx/dt = f(x)
  *
  * \return Next state after integration
  */
template<typename Scalar, typename = typename std::enable_if<std::is_floating_point<Scalar>::value>::type>
Scalar rk4(Scalar x, double dt, Scalar xp) {
  auto derivative_fnc = [](const Scalar d) { return d; };
  // if (!std::isfinite(dt) || dt <= static_cast<Scalar>(0)) {
  //   return x;
  // }
  //const Scalar half_dt = static_cast<Scalar>(0.5) * dt;
  const Scalar k1 = dt * derivative_fnc(xp);
  const Scalar k2 = dt * derivative_fnc(xp + (0.5F * k1));
  const Scalar k3 = dt * derivative_fnc(xp + (0.5F * k2));
  const Scalar k4 = dt * derivative_fnc(xp + k3);
  return x + ((k1 + k4)/6.0F) + ((k2 + k3)/3.0F);
}

/*
 * Safer RK4 overload: integrate x' = f(x) with an explicit derivative function.
 */
// template<typename Scalar, typename DerivFunc,
//          typename = typename std::enable_if<std::is_floating_point<Scalar>::value>::type>
// Scalar rk4(Scalar x, double dt, DerivFunc derivative_fnc) {
//   if (!std::isfinite(dt) || dt <= 0) {
//     return x;
//   }
//   const Scalar k1 = dt * derivative_fnc(x);
//   const Scalar k2 = dt * derivative_fnc(x + ((0.5) * k1));
//   const Scalar k3 = dt * derivative_fnc(x + ((0.5) * k2));
//   const Scalar k4 = dt * derivative_fnc(x + k3);
//   return x + ((k1 + k4) / static_cast<Scalar>(6.0)) + ((k2 + k3) / static_cast<Scalar>(3.0));
// }


/*!
  * \brief 4th order Runge-Kutta integration for general Eigen vectors with non-linear dynamics.
  *
  * \tparam VectorType Eigen vector type (e.g., Vector3f, VectorXf, VectorXd)
  * \tparam DerivFunc Function type that computes derivative: VectorType f(const VectorType& x)
  *
  * \param x Current state vector
  * \param dt Time step
  * \param derivative Function object that computes dx/dt = f(x)
  *
  * \return Next state after integration
  */
template<typename VectorType>
VectorType rk4_eigen(const VectorType& x, double dt, const VectorType& xp) {
  auto derivative_fnc = [](const VectorType& d) { return d; };
    // if (!std::isfinite(dt) || dt <= 0.0F) {
    //     return x;
    // }
    //const float half_dt = 0.5F * dt;
    const VectorType k1 = dt * derivative_fnc(xp);
    const VectorType k2 = dt * derivative_fnc(xp + (0.5F * k1));
    const VectorType k3 = dt * derivative_fnc(xp + (0.5F * k2));
    const VectorType k4 = dt * derivative_fnc(xp + k3);
    return x + ((k1 + k4)/6.0F) + ((k2 + k3)/3.0F);
}

/*
 * Safer RK4 overload: integrate x' = f(x) for Eigen types.
 */
// template<typename VectorType, typename DerivFunc>
// VectorType rk4_eigen(const VectorType& x, double dt, DerivFunc derivative_fnc) {
//   // if (!std::isfinite(dt) || dt <= 0.0F) {
//   //   return x;
//   // }
//   const VectorType k1 = dt * derivative_fnc(x);
//   const VectorType k2 = dt * derivative_fnc(x + (0.5F * k1));
//   const VectorType k3 = dt * derivative_fnc(x + (0.5F * k2));
//   const VectorType k4 = dt * derivative_fnc(x + k3);
//   return x + ((k1 + k4) / 6.0F) + ((k2 + k3) / 3.0F);
// }

/*!
  * \brief 4th order Runge-Kutta integration for general Eigen matrices with non-linear dynamics.
  *
  * Optimized RK4 integrator for arbitrary Eigen matrix types. Supports non-linear dynamics
  * through function objects (lambdas, functors, std::function).
  *
  * \tparam MatrixType Eigen matrix type (e.g., MatrixXf, MatrixXd, Matrix3f)
  * \tparam DerivFunc Function type that computes derivative: MatrixType f(const MatrixType& X)
  *
  * \param X Current state matrix
  * \param derivative Function object that computes dX/dt = f(X)
  * \param dt Time step
  *
  * \return Next state after integration
  */
template<typename MatrixType>
MatrixType rk4_eigen_matrix(const MatrixType& X, double dt, const MatrixType& Xp) {
  auto derivative_fnc = [](const MatrixType& d) { return d; };
  // if (!std::isfinite(dt) || dt <= 0.0F) {
  //   return X;
  // }
  //const float half_dt = 0.5F * dt;
  const MatrixType k1 = dt * derivative_fnc(Xp);
  const MatrixType k2 = dt * derivative_fnc(Xp + (0.5F * k1));
  const MatrixType k3 = dt * derivative_fnc(Xp + (0.5F * k2));
  const MatrixType k4 = dt * derivative_fnc(Xp + k3);
  return X + ((k1 + k4)/6.0F) + ((k2 + k3)/3.0F);
}

/*!
  * \brief 4th order Runge-Kutta integration for Vector3f with non-linear dynamics.
  *
  * \tparam DerivFunc Function type that computes derivative: Vector3f f(const Vector3f& x)
  *
  * \param x Current state vector
  * \param dt Time step
  * \param derivative Function object that computes dx/dt = f(x)
  *
  * \return Next state after integration
  */
// template<typename DerivFunc>
// Eigen::Vector3f rk4_vec(const Eigen::Vector3f& x, float dt, DerivFunc derivative) {
//   return rk4_eigen(x, dt, derivative);
// }

//Optional constant-derivative shortcut (uncomment if you want it).
inline float rk4_const(float x, double dt, float dx) { return x + dt * dx; }
inline Eigen::Vector3f rk4_const(const Eigen::Vector3f& x, double dt, const Eigen::Vector3f& dx) { return x + dt * dx; }

template<typename Derived>
typename Derived::PlainObject rk4_const(const Eigen::MatrixBase<Derived>& x,
                                        double dt,
                                        const Eigen::MatrixBase<Derived>& dx) {
  return x + (dt * dx);
}



/*! \fn sign, función que contiene la función signo.
 */
float sign(const float a);

float sigmoide(const float a, const float d);

float signth(const float a, const float p);

Eigen::Vector3f signth(const Eigen::Vector3f& a, const float p);


class Levant3{
public:
    explicit Levant3(uint8_t _mode = 1, float _L = 1.0F, double _p = 300.0);
    ~Levant3();

    void setParam(double L, double p);

    double compute(double& f, float dt);
    Eigen::Vector3f compute(const Eigen::Vector3f& f, float dt);

    void Reset();

private:
    uint8_t mode;
    double L;
    double p;
    double L_p14 = 0.0; // L^(1/4)
    double L_p13 = 0.0; // L^(1/3)
    double L_p12 = 0.0; // L^(1/2)
    double z0 = 0.0F; // Initial condition for z0
    double z1 = 0.0F; // Initial condition for z1
    double z2 = 0.0F; // Initial condition for z2
    double z3 = 0.0F; // Initial condition for z3

    Eigen::Vector3f nu0;
    Eigen::Vector3f nu1;
    Eigen::Vector3f nu2;
    Eigen::Vector3f z3p;

    Eigen::Vector3f z0_1 = Eigen::Vector3f::Zero(); // Initial condition for z0
    Eigen::Vector3f z1_1 = Eigen::Vector3f::Zero(); // Initial condition for z1
    Eigen::Vector3f z2_1 = Eigen::Vector3f::Zero(); // Initial condition for z2
    Eigen::Vector3f z3_1 = Eigen::Vector3f::Zero(); // Initial condition for z3

    void updateLPowers();
    double sign_(double val);
};



class Levant_diff{
public:
    /*!
    * \brief Constructor for a Levant's derivative.
    *
    * \param mode  Mode of the function, tanh or sign
    * \param alpha Parameter of Levant's derivative
    * \param lamb  Parameter of Levant's derivative
    * \param p     Parameter for tanh function
    */
    Levant_diff(std::string mode = "tanh", const float aplha = 1, const float lamb = 1, const float p = 3000);
    ~Levant_diff();

    /*!
    * \brief Set the parameters of the Levant's derivative.
    * 
    * \param alpha Parameter of Levant's derivative
    * \param lamb  Parameter of Levant's derivative
    * \param p     Parameter for tanh function
    * 
    */
    void setParam(const float alpha, const float lamb, const float p);

    /*!
    * \brief Set the parameters of the Levant's derivative.
    * 
    * \param alpha Parameter of Levant's derivative
    * \param lamb  Parameter of Levant's derivative
    * 
    */
    void setParam(const float alpha, const float lamb);

    /*!
    * \brief Set the parameters of the Levant's derivative.
    * 
    * \param alpha Parameter of Levant's derivative
    * \param lamb  Parameter of Levant's derivative
    * 
    */
    void setParam_vec(const Eigen::Vector3f& alpha, const Eigen::Vector3f& lamb);

    /*!
    * \brief Reset.
    */
    void Reset(void);

    /*!
    * \brief Compute the Levant's derivative.
    *
    * \param f  Input signal
    * \param dt Time step
    * 
    * \return Derivative of the input signal
    */
    float Compute(const float& f, const float dt);

    /*!
    *
    * \brief Compute the Levant's derivative. 
    * 
    * \param f  Input signal
    * \param dt Time step
    * 
    */
    void Compute(float &u, const float& f, const float dt);

    /*!
    * \brief Compute the Levant's derivative.
    *
    * \param f  Input signal
    * \param dt Time step
    * 
    * \return Derivative of the input signal
    */
    Eigen::Vector3f Compute(const Eigen::Vector3f& f, const float dt);

    /*!
    * \brief Get the error.
    *
    * \return Error
    */
    float getErr(void);

    /*!
    * \brief Get the error.
    *
    * \return Error
    */
    Eigen::Vector3f getErr_v(void);

private:
    std::string mode;
    float alpha, lamb, p;
    float u, u1, u1p, x;

    Eigen::Vector3f alpha_vec, lamb_vec;

    float err;
    Eigen::Vector3f err_v;

    Eigen::Vector3f u_vec, u1_vec, u1p_vec, x_vec;

    float sign_(const float a);

};

#endif // NMETHODS_H
