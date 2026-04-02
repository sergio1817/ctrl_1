/*!
 * \file NMethods.h
 * \brief Numerical Methods Library — IRED differentiator + robust integration
 * \author Sergio Urzua, et al. (alumni of RYMA)
 * \date 2023/05/18
 * \version 2.0
 *
 * GCC 4.9 / C++11 ONLY.  No C++14/17 features.
 */

#ifndef NMETHODS_H
#define NMETHODS_H

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <string>
#include <type_traits>

/* ====================================================================
 *  1.  FORWARD-EULER  (formerly "rk4_const")
 * ==================================================================== */

/*! \brief Forward-Euler integration for scalar floating-point types. */
template<typename Scalar,
         typename = typename std::enable_if<std::is_floating_point<Scalar>::value>::type>
Scalar euler_integrate(Scalar x, double dt, Scalar dx) {
    return x + static_cast<Scalar>(dt) * dx;
}

/*! \brief Forward-Euler integration for Eigen matrix/vector types. */
template<typename Derived>
typename Derived::PlainObject
euler_integrate(const Eigen::MatrixBase<Derived>& x,
                double dt,
                const Eigen::MatrixBase<Derived>& dx) {
    return (x + dt * dx).eval();
}

/* ---------- backward-compatible aliases ---------- */
inline float rk4_const(float x, double dt, float dx) {
    return euler_integrate(x, dt, dx);
}
inline double rk4_const(double x, double dt, double dx) {
    return euler_integrate(x, dt, dx);
}
inline Eigen::Vector3f rk4_const(const Eigen::Vector3f& x, double dt,
                                 const Eigen::Vector3f& dx) {
    return euler_integrate(x, dt, dx);
}

template<typename Derived>
typename Derived::PlainObject
rk4_const(const Eigen::MatrixBase<Derived>& x,
          double dt,
          const Eigen::MatrixBase<Derived>& dx) {
    return euler_integrate(x, dt, dx);
}

/* ====================================================================
 *  2.  KAHAN-COMPENSATED EULER INTEGRATION
 * ==================================================================== */

/*! \brief Kahan summation state — primary template for scalar types. */
template<typename T, typename Enable = void>
struct KahanState {
    T value;
    T compensation;
    KahanState() : value(T()), compensation(T()) {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*! \brief KahanState specialisation for Eigen::Matrix types. */
template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct KahanState<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>, void> {
    typedef Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> MatType;
    MatType value;
    MatType compensation;
    KahanState() {
        value.setZero();
        compensation.setZero();
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*! \brief Kahan-compensated Euler integration — scalar. */
template<typename Scalar>
typename std::enable_if<std::is_floating_point<Scalar>::value>::type
kahan_integrate(KahanState<Scalar>& state, double dt, Scalar dx) {
    Scalar y = static_cast<Scalar>(dt) * dx - state.compensation;
    Scalar t = state.value + y;
    state.compensation = (t - state.value) - y;
    state.value = t;
}

/*! \brief Kahan-compensated Euler integration — Eigen matrix/vector. */
template<typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
void kahan_integrate(
        KahanState<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> >& state,
        double dt,
        const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& dx) {
    typedef Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> MatType;
    MatType y = (static_cast<Scalar>(dt) * dx - state.compensation).eval();
    MatType t = (state.value + y).eval();
    state.compensation = (t - state.value) - y;
    state.value = t;
}

/* ====================================================================
 *  3.  IMPLICIT MIDPOINT  (fixed-point, niter iterations)
 * ==================================================================== */

/*!
 * \brief Implicit-midpoint integrator with fixed-point iterations.
 *
 * Computes  x_{k+1} = x_k + dt * f( (x_k + x_{k+1}) / 2 )
 * using \p niter fixed-point iterations starting from x_mid = x_k.
 *
 * \tparam StateType   float, double, or any Eigen matrix type
 * \tparam DerivFunc   callable  StateType f(const StateType&)
 */
template<typename StateType, typename DerivFunc>
StateType implicit_midpoint(const StateType& x, double dt, DerivFunc f, int niter = 3) {
    StateType x_mid = x;
    for (int i = 0; i < niter; ++i) {
        x_mid = x + (0.5 * dt) * f(x_mid);
    }
    return static_cast<StateType>( 2.0 * x_mid - x );
}

/* ====================================================================
 *  4.  CLASSIC  RK4
 * ==================================================================== */

/* --- 4a.  Constant-derivative RK4 (backward compat — same as old code) --- */
template<typename Scalar,
         typename = typename std::enable_if<std::is_floating_point<Scalar>::value>::type>
Scalar rk4(Scalar x, double dt, Scalar xp) {
    Scalar k1 = static_cast<Scalar>(dt) * xp;
    Scalar k2 = static_cast<Scalar>(dt) * (xp + static_cast<Scalar>(0.5) * k1);
    Scalar k3 = static_cast<Scalar>(dt) * (xp + static_cast<Scalar>(0.5) * k2);
    Scalar k4 = static_cast<Scalar>(dt) * (xp + k3);
    return x + (k1 + k4) / static_cast<Scalar>(6.0)
             + (k2 + k3) / static_cast<Scalar>(3.0);
}

template<typename VectorType>
VectorType rk4_eigen(const VectorType& x, double dt, const VectorType& xp) {
    typedef typename VectorType::Scalar S;
    VectorType k1 = static_cast<S>(dt) * xp;
    VectorType k2 = static_cast<S>(dt) * (xp + static_cast<S>(0.5) * k1);
    VectorType k3 = static_cast<S>(dt) * (xp + static_cast<S>(0.5) * k2);
    VectorType k4 = static_cast<S>(dt) * (xp + k3);
    return x + (k1 + k4) / static_cast<S>(6.0)
             + (k2 + k3) / static_cast<S>(3.0);
}

template<typename MatrixType>
MatrixType rk4_eigen_matrix(const MatrixType& X, double dt, const MatrixType& Xp) {
    typedef typename MatrixType::Scalar S;
    MatrixType k1 = static_cast<S>(dt) * Xp;
    MatrixType k2 = static_cast<S>(dt) * (Xp + static_cast<S>(0.5) * k1);
    MatrixType k3 = static_cast<S>(dt) * (Xp + static_cast<S>(0.5) * k2);
    MatrixType k4 = static_cast<S>(dt) * (Xp + k3);
    return X + (k1 + k4) / static_cast<S>(6.0)
             + (k2 + k3) / static_cast<S>(3.0);
}

/* --- 4b.  Proper RK4 with derivative function --- */
template<typename Scalar, typename DerivFunc>
typename std::enable_if<std::is_floating_point<Scalar>::value, Scalar>::type
rk4_func(Scalar x, double dt, DerivFunc f) {
    Scalar k1 = static_cast<Scalar>(dt) * f(x);
    Scalar k2 = static_cast<Scalar>(dt) * f(x + static_cast<Scalar>(0.5) * k1);
    Scalar k3 = static_cast<Scalar>(dt) * f(x + static_cast<Scalar>(0.5) * k2);
    Scalar k4 = static_cast<Scalar>(dt) * f(x + k3);
    return x + (k1 + k4) / static_cast<Scalar>(6.0)
             + (k2 + k3) / static_cast<Scalar>(3.0);
}

template<typename VectorType, typename DerivFunc>
VectorType rk4_eigen_func(const VectorType& x, double dt, DerivFunc f) {
    typedef typename VectorType::Scalar S;
    VectorType k1 = static_cast<S>(dt) * f(x);
    VectorType k2 = static_cast<S>(dt) * f(x + static_cast<S>(0.5) * k1);
    VectorType k3 = static_cast<S>(dt) * f(x + static_cast<S>(0.5) * k2);
    VectorType k4 = static_cast<S>(dt) * f(x + k3);
    return x + (k1 + k4) / static_cast<S>(6.0)
             + (k2 + k3) / static_cast<S>(3.0);
}

/* ====================================================================
 *  5.  BOGACKI-SHAMPINE 3(2)  — adaptive step
 * ==================================================================== */

template<typename StateType>
struct BS32Result {
    StateType x_next;
    double error_estimate;
    double dt_suggested;
};

template<typename StateType, typename DerivFunc>
BS32Result<StateType> bogacki_shampine_32(const StateType& x, double dt,
                                          DerivFunc f, double tol = 1e-6) {
    typedef typename StateType::Scalar S;
    StateType k1 = f(x);
    StateType k2 = f(x + static_cast<S>(0.5  * dt) * k1);
    StateType k3 = f(x + static_cast<S>(0.75 * dt) * k2);

    StateType x3 = x + static_cast<S>(dt) *
                   (static_cast<S>(2.0/9.0) * k1
                  + static_cast<S>(1.0/3.0) * k2
                  + static_cast<S>(4.0/9.0) * k3);

    StateType k4 = f(x3);

    StateType x2 = x + static_cast<S>(dt) *
                   (static_cast<S>(7.0/24.0)  * k1
                  + static_cast<S>(1.0/4.0)   * k2
                  + static_cast<S>(1.0/3.0)   * k3
                  + static_cast<S>(1.0/8.0)   * k4);

    double err = (x3 - x2).norm();
    double dt_new = dt;
    if (err > 1e-15) {
        dt_new = 0.9 * dt * std::pow(tol / err, 1.0 / 3.0);
    }

    BS32Result<StateType> result;
    result.x_next = x3;
    result.error_estimate = err;
    result.dt_suggested = dt_new;
    return result;
}

/* ====================================================================
 *  6.  SIGN FUNCTIONS
 * ==================================================================== */

float sign(const float a);
float sigmoide(const float a, const float d);
float signth(const float a, const float p);
Eigen::Vector3f signth(const Eigen::Vector3f& a, const float p);

/* ====================================================================
 *  7.  LEGACY FUNCTION DECLARATIONS
 * ==================================================================== */

float rk4o(float(*fPtr)(float), float iC, const float iCdt, const float dt);
float function1d(float iCdt);

/* ====================================================================
 *  8.  IRED  (Implicit Robust Exact Differentiator)  — Seeber 2024
 * ==================================================================== */

/*!
 * \brief Implicit Robust Exact Differentiator of order \p Order.
 *
 * Template parameter Order is the differentiation order (1..4).
 * Provides both scalar (double) and Vector3f interfaces.
 *
 * Reference: Seeber 2024, arXiv 2404.02770
 */
template<int Order>
class IRED {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * \param L           Lipschitz constant of the (m+1)-th derivative of the signal.
     * \param ema_alpha   Exponential moving average coefficient for the highest
     *                    derivative output (0 < alpha <= 1).  alpha=1 means no
     *                    filtering.  Lower values smooth the inherent Nyquist
     *                    oscillation of z_{m+1} at the sliding boundary.
     *                    Recommended: 0.1–0.3 for noisy signals, 1.0 for clean.
     */
    explicit IRED(double L = 1.0, double ema_alpha = 0.2);
    ~IRED();

    /*! \brief Scalar step. */
    void step(double measurement, double dt);
    /*! \brief Get scalar output.  derivative_order: 0 = 1st deriv, 1 = 2nd, ... */
    double output(int derivative_order) const;

    /*! \brief Vector3f step (3 independent scalar channels). */
    void step(const Eigen::Vector3f& measurement, float dt);
    /*! \brief Get Vector3f output. */
    Eigen::Vector3f output_vec(int derivative_order) const;

    void reset();
    void setLipschitz(double L);
    void setEmaAlpha(double alpha);  ///< Set EMA filter coefficient (0 < alpha <= 1)

private:
    double L_;
    double ema_alpha_;               ///< EMA coefficient for highest derivative
    double lambda_[Order + 1];       ///< gains
    double z_[Order + 1];            ///< scalar states  z[0] = position, z[Order] = highest
    double c_[Order][Order];         ///< output correction coefficients  (0-indexed)

    /* EMA filter state for the highest derivative (per channel) */
    double ema_;                     ///< scalar channel EMA state
    double ema_vec_[3];              ///< vector channel EMA states

    /* three independent scalar channels for the vector interface */
    double z_vec_[3][Order + 1];

    void compute_gains();
    void compute_c_coeffs();
    double newton_solve(double abs_b, double T) const;

    /* internal step for a single scalar channel */
    void step_scalar(double* z, double measurement, double T);
    /* internal output for a single scalar channel */
    double output_scalar(const double* z, int derivative_order, double T_last,
                         double ema_state) const;

    double T_last_;                  ///< last dt used (for output correction)
    float  T_last_f_;                ///< float copy for vector interface
};

/* ====================================================================
 *  9.  LEVANT DIFFERENTIATORS  (legacy)
 * ==================================================================== */

class Levant_diff {
public:
    Levant_diff(std::string mode = "tanh", const float aplha = 1,
                const float lamb = 1, const float p = 3000);
    ~Levant_diff();

    void setParam(const float alpha, const float lamb, const float p);
    void setParam(const float alpha, const float lamb);
    void setParam_vec(const Eigen::Vector3f& alpha, const Eigen::Vector3f& lamb);
    void Reset(void);

    float Compute(const float& f, const float dt);
    void  Compute(float& u, const float& f, const float dt);
    Eigen::Vector3f Compute(const Eigen::Vector3f& f, const float dt);

    float getErr(void);
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


/*!
 * \brief Levant3 — 3rd-order differentiator (legacy wrapper around IRED<3>).
 *
 * Preserves the old public interface.  Internally delegates to IRED<3>.
 */
class Levant3 {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     * \param _mode       0 = sign, 1 = tanh (legacy, not used by IRED).
     * \param _L          Lipschitz constant of the 4th derivative of the signal.
     * \param _ema_alpha  EMA filter coefficient for highest derivative (0.1–1.0).
     */
    explicit Levant3(uint8_t _mode = 1, float _L = 1.0F, double _ema_alpha = 0.2);
    ~Levant3();

    /*!
     * \brief Set parameters.
     * \param L          Lipschitz constant (passed to IRED).
     * \param ema_alpha  EMA filter coefficient for highest derivative.
     */
    void setParam(double L, double ema_alpha = 0.2);

    double compute(double& f, float dt);
    Eigen::Vector3f compute(const Eigen::Vector3f& f, float dt);

    void Reset();

    /*! \brief Access the underlying IRED<3> instance. */
    IRED<3>& ired() { return ired_; }

private:
    uint8_t mode;
    double L;
    IRED<3> ired_;
};

#endif // NMETHODS_H
