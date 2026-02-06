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
#include <cstdint>
#include <utility>
#include <functional>


/*!
  * \brief 4th order Runge-Kutta integration for general Eigen vectors with non-linear dynamics.
  *
  * Optimized RK4 integrator for arbitrary Eigen vector types. Supports non-linear dynamics
  * through function objects (lambdas, functors, std::function).
  *
  * \tparam VectorType Eigen vector type (e.g., Vector3f, VectorXf, VectorXd)
  * \tparam DerivFunc Function type that computes derivative: VectorType f(const VectorType& x)
  *
  * \param x Current state vector
  * \param derivative Function object that computes dx/dt = f(x)
  * \param dt Time step
  *
  * \return Next state after integration
  */
template<typename VectorType>
VectorType rk4_eigen(const VectorType& x, const VectorType& xp, float dt) {
    auto derivative = [](const VectorType& v) { return  v; };
    const VectorType k1 = derivative(xp) * dt;
    const VectorType k2 = derivative(xp + (k1/2.0F)) * dt;
    const VectorType k3 = derivative(xp + (k2/2.0F)) * dt;
    const VectorType k4 = derivative(xp + k3)* dt;
    return x + ((k1 + k4)/6.0F) + ((k2 + k3)/3.0F);
}

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
MatrixType rk4_eigen_matrix(const MatrixType& X, const MatrixType& Xp, float dt) {
    auto derivative = [](const MatrixType& M) {return M;};
    const MatrixType k1 = derivative(Xp) * dt;
    const MatrixType k2 = derivative(Xp + (k1/2.0F)) * dt;
    const MatrixType k3 = derivative(Xp + (k2/2.0F)) * dt;
    const MatrixType k4 = derivative(Xp + k3)* dt;
    return X + ((k1 + k4)/6.0F) + ((k2 + k3)/3.0F);
}

/*!
  * \brief Optimized 4th order Runge-Kutta for Vector3f with non-linear dynamics.
  *
  * Specialized version for 3D vectors. More efficient than the generic template.
  * Supports non-linear dynamics where derivative can depend on current state.
  *
  * \param x Current state vector (3D)
  * \param derivative Function object that computes dx/dt = f(x)
  * \param dt Time step
  *
  * \return Next state after integration
  */
Eigen::Vector3f rk4_vec3f(const Eigen::Vector3f& x, 
                          std::function<Eigen::Vector3f(const Eigen::Vector3f&)> derivative, 
                          float dt);

/*!
  * \brief Optimized 4th order Runge-Kutta for dynamic-sized vectors with non-linear dynamics.
  *
  * For variable-size vectors. Supports non-linear dynamics.
  *
  * \param x Current state vector
  * \param derivative Function object that computes dx/dt = f(x)
  * \param dt Time step
  *
  * \return Next state after integration
  */
Eigen::VectorXf rk4_vecXf(const Eigen::VectorXf& x, 
                          std::function<Eigen::VectorXf(const Eigen::VectorXf&)> derivative, 
                          float dt);

#include <functional>


/*!
  * \brief 4th order Runge-Kutta integration for general Eigen vectors with non-linear dynamics.
  *
  * Optimized RK4 integrator for arbitrary Eigen vector types. Supports non-linear dynamics
  * through function objects (lambdas, functors, std::function).
  *
  * \tparam VectorType Eigen vector type (e.g., Vector3f, VectorXf, VectorXd)
  * \tparam DerivFunc Function type that computes derivative: VectorType f(const VectorType& x)
  *
  * \param x Current state vector
  * \param derivative Function object that computes dx/dt = f(x)
  * \param dt Time step
  *
  * \return Next state after integration
  */
template<typename VectorType, typename DerivFunc>
VectorType rk4_eigen(const VectorType& x, DerivFunc derivative, float dt) {
    const VectorType k1 = derivative(x);
    const VectorType k2 = derivative(x + 0.5f * dt * k1);
    const VectorType k3 = derivative(x + 0.5f * dt * k2);
    const VectorType k4 = derivative(x + dt * k3);
    return x + (dt / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);
}

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
template<typename MatrixType, typename DerivFunc>
MatrixType rk4_eigen_matrix(const MatrixType& X, DerivFunc derivative, float dt) {
    const MatrixType k1 = derivative(X);
    const MatrixType k2 = derivative(X + 0.5f * dt * k1);
    const MatrixType k3 = derivative(X + 0.5f * dt * k2);
    const MatrixType k4 = derivative(X + dt * k3);
    return X + (dt / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);
}

/*!
  * \brief Optimized 4th order Runge-Kutta for Vector3f with non-linear dynamics.
  *
  * Specialized version for 3D vectors. More efficient than the generic template.
  * Supports non-linear dynamics where derivative can depend on current state.
  *
  * \param x Current state vector (3D)
  * \param derivative Function object that computes dx/dt = f(x)
  * \param dt Time step
  *
  * \return Next state after integration
  */
Eigen::Vector3f rk4_vec3f(const Eigen::Vector3f& x, 
                          std::function<Eigen::Vector3f(const Eigen::Vector3f&)> derivative, 
                          float dt);

/*!
  * \brief Optimized 4th order Runge-Kutta for dynamic-sized vectors with non-linear dynamics.
  *
  * For variable-size vectors. Supports non-linear dynamics.
  *
  * \param x Current state vector
  * \param derivative Function object that computes dx/dt = f(x)
  * \param dt Time step
  *
  * \return Next state after integration
  */
Eigen::VectorXf rk4_vecXf(const Eigen::VectorXf& x, 
                          std::function<Eigen::VectorXf(const Eigen::VectorXf&)> derivative, 
                          float dt);

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
float rk4(float(*fPtr)(float),  float iC, const float iCdt, const float dt);

/*!
  * \brief 4 order Runge-Kutta integration method.
  *
  * Compute the integral of a function using the Runge-Kutta method of 4th order for a Eigen::Vector3d object.
  *
  * \param iC   Integral of the function
  * \param iCdt Derivative of the integral of the function
  * \param dt   Time step
  * 
  * \return Integral of the function
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


class Levant3{
public:
    explicit Levant3(uint8_t _mode = 1, float _L = 1.0F, double _p = 300.0);
    ~Levant3();

    void setParam(double L, double p);

    double compute(double f, float dt);
    Eigen::Vector3f compute(const Eigen::Vector3f f, float dt);

    void Reset();

private:
    uint8_t mode;
    double L;
    double p;
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
    void setParam_vec(const Eigen::Vector3f alpha, const Eigen::Vector3f lamb);

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
    float Compute(const float f, const float dt);

    /*!
    *
    * \brief Compute the Levant's derivative. 
    * 
    * \param f  Input signal
    * \param dt Time step
    * 
    */
    void Compute(float &u, const float f, const float dt);

    /*!
    * \brief Compute the Levant's derivative.
    *
    * \param f  Input signal
    * \param dt Time step
    * 
    * \return Derivative of the input signal
    */
    Eigen::Vector3f Compute(const Eigen::Vector3f f, const float dt);

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
