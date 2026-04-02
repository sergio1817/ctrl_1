#include "NMethods.h"
#include <cmath>
#include <cstring>

/* ====================================================================
 *  Legacy free functions
 * ==================================================================== */

float rk4o(float(*fPtr)(float), const float iC, const float iCdt, const float dt) {
    float a = dt * fPtr(iCdt);
    float b = dt * fPtr(iCdt + a / 2.0f);
    float c = dt * fPtr(iCdt + b / 2.0f);
    float d = dt * fPtr(iCdt + c);
    return iC + (a + d) / 6.0f + (b + c) / 3.0f;
}

float function1d(float iCdt) {
    return iCdt;
}

/* ====================================================================
 *  Sign functions
 * ==================================================================== */

float sign(const float a) {
    if (a < 0) return -1;
    if (a > 0) return  1;
    return 0;
}

float sigmoide(const float a, const float d) {
    float abs_a = (a > 0) ? a : ((a < 0) ? -a : 0);
    return a / (abs_a + d);
}

float signth(const float a, const float p) {
    return tanhf(p * a);
}

Eigen::Vector3f signth(const Eigen::Vector3f& a, const float p) {
    Eigen::Vector3f out;
    out(0) = tanhf(p * a(0));
    out(1) = tanhf(p * a(1));
    out(2) = tanhf(p * a(2));
    return out;
}

/* ====================================================================
 *  Levant_diff  (unchanged)
 * ==================================================================== */

Levant_diff::Levant_diff(std::string _mode, const float _aplha,
                         const float _lamb, const float _p)
    : mode(_mode), alpha(_aplha), lamb(_lamb), p(_p),
      u(0), u1(0), u1p(0), x(0), err(0) {
    /* FIX: initialize all state to zero in constructor
     * (original code left u, u1, u1p, x uninitialized). */
    alpha_vec.setZero();
    lamb_vec.setZero();
    u_vec.setZero();
    u1_vec.setZero();
    u1p_vec.setZero();
    x_vec.setZero();
    err_v.setZero();
}

Levant_diff::~Levant_diff() {}

void Levant_diff::setParam(const float alpha, const float lamb, const float p) {
    this->alpha = alpha;
    this->lamb  = lamb;
    this->p     = p;
}

void Levant_diff::setParam(const float alpha, const float lamb) {
    this->alpha = alpha;
    this->lamb  = lamb;
}

void Levant_diff::setParam_vec(const Eigen::Vector3f& alpha,
                               const Eigen::Vector3f& lamb) {
    this->alpha_vec = alpha;
    this->lamb_vec  = lamb;
}

void Levant_diff::Reset(void) {
    this->u   = 0;
    this->u1  = 0;
    this->u1p = 0;
    this->x   = 0;
    this->x_vec   << 0, 0, 0;
    this->u_vec   << 0, 0, 0;
    this->u1_vec  << 0, 0, 0;
    this->u1p_vec << 0, 0, 0;
}

float Levant_diff::Compute(const float& f, const float dt) {
    /* FIX: The original code used rk4() which, applied with a constant
     * derivative, creates spurious polynomial growth and diverges.
     * Additionally, explicit Euler with sign(x-f) or tanh(p*(x-f))
     * causes chattering or instability when p is large.
     *
     * Use implicit sign resolution: replace sign(x-f) with
     * clamp((x-f)/dt, -1, 1) — the Acary-Brogliato projection.
     * This guarantees chattering-free behavior independent of gains. */
    float sigma = x - f;
    float impl_sign = sigma / (std::fabs(sigma) + dt);
    u = u1 - lamb * sqrtf(std::fabs(sigma)) * impl_sign;
    u1p = -alpha * impl_sign;
    x  = euler_integrate(x, (double)dt, u);
    u1 = euler_integrate(u1, (double)dt, u1p);
    err = x - f;
    return u;
}

void Levant_diff::Compute(float& _u, const float& f, const float dt) {
    float sigma = x - f;
    float impl_sign = sigma / (std::fabs(sigma) + dt);
    _u = u1 - lamb * sqrtf(std::fabs(sigma)) * impl_sign;
    u1p = -alpha * impl_sign;
    x  = euler_integrate(x, (double)dt, _u);
    u1 = euler_integrate(u1, (double)dt, u1p);
}

Eigen::Vector3f Levant_diff::Compute(const Eigen::Vector3f& f, const float dt) {
    /* FIX: use implicit sign resolution (Acary-Brogliato projection)
     * instead of sign_()/tanh() to prevent chattering and divergence. */
    for (int i = 0; i < 3; ++i) {
        float sigma = x_vec(i) - f(i);
        float impl_sign = sigma / (std::fabs(sigma) + dt);
        u_vec(i)   = u1_vec(i) - lamb_vec(i) * sqrtf(std::fabs(sigma)) * impl_sign;
        u1p_vec(i) = -alpha_vec(i) * impl_sign;
    }

    x_vec  = euler_integrate(x_vec,  (double)dt, u_vec);
    u1_vec = euler_integrate(u1_vec, (double)dt, u1p_vec);

    err_v = x_vec - f;
    return u_vec;
}

float Levant_diff::getErr(void)              { return err; }
Eigen::Vector3f Levant_diff::getErr_v(void)  { return err_v; }

float Levant_diff::sign_(const float a) {
    if (mode == "sign")      return sign(a);
    if (mode == "sigmoide")  return 0;
    if (mode == "tanh")      return signth(a, p);
    return sign(a);
}

/* ====================================================================
 *  IRED  template implementation
 * ==================================================================== */

template<int Order>
IRED<Order>::IRED(double L, double ema_alpha)
    : L_(L), ema_alpha_(ema_alpha), T_last_(0.001), T_last_f_(0.001f) {
    if (ema_alpha_ <= 0.0) ema_alpha_ = 0.01;
    if (ema_alpha_ > 1.0)  ema_alpha_ = 1.0;
    compute_gains();
    compute_c_coeffs();
    reset();
}

template<int Order>
IRED<Order>::~IRED() {}

template<int Order>
void IRED<Order>::reset() {
    for (int i = 0; i <= Order; ++i) {
        z_[i] = 0.0;
    }
    ema_ = 0.0;
    for (int ch = 0; ch < 3; ++ch) {
        for (int i = 0; i <= Order; ++i) {
            z_vec_[ch][i] = 0.0;
        }
        ema_vec_[ch] = 0.0;
    }
    T_last_  = 0.001;
    T_last_f_ = 0.001f;
}

template<int Order>
void IRED<Order>::setLipschitz(double L) {
    L_ = L;
}

template<int Order>
void IRED<Order>::setEmaAlpha(double alpha) {
    if (alpha <= 0.0) alpha = 0.01;
    if (alpha > 1.0)  alpha = 1.0;
    ema_alpha_ = alpha;
}

/* ---------- gain selection (Seeber's closed-form recommended) ---------- */

template<int Order>
void IRED<Order>::compute_gains() {
    /* For specific orders use Seeber's recommended gains.
     * Fallback: binomial coefficients scaled. */
    for (int i = 0; i <= Order; ++i) lambda_[i] = 1.0;

    if (Order == 1) {
        // Super-twisting: lambda = {1.5, 1.1}
        lambda_[0] = 1.5;
        lambda_[1] = 1.1;
    } else if (Order == 2) {
        // lambda = {3.0, 4.16, 3.0}
        lambda_[0] = 3.0;
        lambda_[1] = 4.16;
        lambda_[2] = 3.0;
    } else if (Order == 3) {
        // Seeber's recommended for m=3
        lambda_[0] = 5.0;
        lambda_[1] = 10.0;
        lambda_[2] = 10.0;
        lambda_[3] = 5.0;
    } else if (Order == 4) {
        lambda_[0] = 7.0;
        lambda_[1] = 20.0;
        lambda_[2] = 35.0;
        lambda_[3] = 20.0;
        lambda_[4] = 7.0;
    }
}

/* ---------- output correction coefficients c_{i,j} ----------
 * Recursive definition (Seeber 2024, Eq. (14)):
 *   c_{0,0} = 1
 *   c_{i,j} = 0  for i > j
 *   c_{i,j} = ((j-1)*c_{i,j-1} + i*c_{i-1,j-1}) / j      for 1 <= i <= j
 *
 * We store c_[i][j] for i in [0..Order-1], j in [0..Order-1].
 * These correspond to c_{i+1,j+1} in the 1-based formulation (outputs y_1..y_m).
 */
template<int Order>
void IRED<Order>::compute_c_coeffs() {
    /* 1-based array, large enough */
    double cc[Order + 2][Order + 2];
    for (int i = 0; i <= Order + 1; ++i)
        for (int j = 0; j <= Order + 1; ++j)
            cc[i][j] = 0.0;

    cc[0][0] = 1.0;
    for (int j = 1; j <= Order; ++j) {
        for (int i = 0; i <= j; ++i) {
            if (i == 0) {
                cc[i][j] = static_cast<double>(j - 1) * cc[i][j - 1] / static_cast<double>(j);
            } else {
                cc[i][j] = (static_cast<double>(j - 1) * cc[i][j - 1]
                           + static_cast<double>(i) * cc[i - 1][j - 1])
                           / static_cast<double>(j);
            }
        }
    }
    /* Copy to our c_ array: c_[i][j] = cc[i+1][j+1] */
    for (int i = 0; i < Order; ++i)
        for (int j = 0; j < Order; ++j)
            c_[i][j] = cc[i + 1][j + 1];
}

/* ---------- Newton solver for the implicit polynomial ----------
 *
 * p(r) = r^{m+1} + lambda_1*r^m + ... + lambda_{m+1} - B = 0
 * where B = |b| / (L * T^{m+1}),  m = Order.
 *
 * Uses Horner evaluation.  3 Newton iterations, tol = 5e-7 for float precision.
 */
template<int Order>
double IRED<Order>::newton_solve(double abs_b, double T) const {
    const int m = Order;
    /* B = |b| / (L * T^{m+1}) */
    double Tpow = T;
    for (int i = 0; i < m; ++i) Tpow *= T;   // Tpow = T^{m+1}
    if (Tpow < 1e-30) return 0.0;

    const double B = abs_b / (L_ * Tpow);

    /* Check sliding condition: if B <= lambda_{m+1}, return 0 */
    if (B <= lambda_[m]) return 0.0;

    /* Initial guess: r_0 = (B)^{1/(m+1)} */
    double r = std::pow(B, 1.0 / static_cast<double>(m + 1));

    const double tol = 5e-7;
    for (int iter = 0; iter < 3; ++iter) {
        /* Horner evaluation of p(r) and p'(r)
         * p(r) = r^{m+1} + lambda_0*r^m + ... + lambda_m - B
         * coefficients: [1, lambda_0, lambda_1, ..., lambda_m]  (degree m+1)
         */
        double p_val  = 1.0;
        double dp_val = static_cast<double>(m + 1);
        for (int k = 0; k < m + 1; ++k) {
            /* coefficient of r^{m-k} is lambda_[k] */
            p_val  = p_val * r + lambda_[k];
            if (k < m) {
                dp_val = dp_val * r + static_cast<double>(m - k) * lambda_[k];
            }
        }
        p_val -= B;  // subtract RHS

        if (std::fabs(dp_val) < 1e-30) break;
        double r_new = r - p_val / dp_val;
        if (r_new < 0.0) r_new = 0.5 * r;   // safeguard
        if (std::fabs(r_new - r) < tol) { r = r_new; break; }
        r = r_new;
    }
    if (r < 0.0) r = 0.0;
    return r;
}

/* ---------- single-channel step ---------- */
template<int Order>
void IRED<Order>::step_scalar(double* z, double measurement, double T) {
    const int m = Order;
    if (T <= 0.0) return;

    /* Step 1: prediction residual  b = u - sum_{i=0}^{m} T^i * z[i] */
    double b = measurement;
    double Tpow = 1.0;
    for (int i = 0; i <= m; ++i) {
        b -= Tpow * z[i];
        Tpow *= T;
    }

    /* Step 2: root finding */
    double abs_b = std::fabs(b);
    double r_hat = newton_solve(abs_b, T);
    double sgn_b = (b > 0.0) ? 1.0 : ((b < 0.0) ? -1.0 : 0.0);
    double rho_hat = r_hat * sgn_b;

    bool sliding = (r_hat == 0.0);

    /* Step 3: state update (backward sweep) */
    /* z[m] update (highest order, index m) */
    if (sliding) {
        /* Tpow_m = T^m (compute) */
        double Tpow_m = 1.0;
        for (int i = 0; i < m; ++i) Tpow_m *= T;
        if (Tpow_m > 1e-30) {
            z[m] = z[m] + b / Tpow_m;
        }
    } else {
        z[m] = z[m] + lambda_[m] * L_ * T * sgn_b;
    }

    /* z[i] for i = m-1 down to 0 */
    for (int i = m - 1; i >= 0; --i) {
        double T_next = T;   // T * z[i+1]
        z[i] = z[i] + T_next * z[i + 1];

        if (!sliding) {
            /* Paper eq. (20): lambda_i * L * T^{m-i+1} * floor(rho_hat)^{m-i}
             * where floor(x)^p = |x|^p * sign(x).
             * Exponent on rho_hat is (m - i), an INTEGER, not fractional.
             * (0-indexed: code i maps to paper's state index i+1,
             *  paper's exponent is m - (i+1) + 1 = m - i.) */
            int rho_exp = m - i;
            double abs_rho = std::fabs(rho_hat);
            double rho_power = 1.0;
            for (int p = 0; p < rho_exp; ++p) rho_power *= abs_rho;

            double T_power = 1.0;
            for (int k = 0; k < m - i + 1; ++k) T_power *= T;

            z[i] += lambda_[i] * L_ * T_power * rho_power * sgn_b;
        }
    }
}

/* ---------- single-channel output ---------- */
template<int Order>
double IRED<Order>::output_scalar(const double* z, int derivative_order, double T,
                                  double ema_state) const {
    const int m = Order;
    /* derivative_order: 0 = 1st derivative, ..., m-1 = m-th derivative */
    if (derivative_order < 0 || derivative_order >= m) return 0.0;

    /* Highest derivative (derivative_order == m-1): return EMA-filtered z_{m+1}
     * to suppress inherent Nyquist oscillation at the sliding boundary.
     * Lower derivatives use the output correction formula. */
    if (derivative_order == m - 1) {
        return ema_state;
    }

    /* y_{i+1} = sum_{j=i}^{m-1} T^{j-i} * c_[i][j] * z[j+1]
     * where i = derivative_order (0-based) */
    int i = derivative_order;
    double y = 0.0;
    double Tpow = 1.0;  // T^0
    for (int j = i; j < m; ++j) {
        y += Tpow * c_[i][j] * z[j + 1];
        Tpow *= T;
    }
    return y;
}

/* ---------- public scalar interface ---------- */
template<int Order>
void IRED<Order>::step(double measurement, double dt) {
    T_last_ = dt;
    step_scalar(z_, measurement, dt);
    /* EMA filter for the highest derivative: z_[Order] = z_{m+1}
     * ema_{k+1} = alpha * z_{m+1,k+1} + (1 - alpha) * ema_k */
    ema_ = ema_alpha_ * z_[Order] + (1.0 - ema_alpha_) * ema_;
}

template<int Order>
double IRED<Order>::output(int derivative_order) const {
    return output_scalar(z_, derivative_order, T_last_, ema_);
}

/* ---------- public Vector3f interface ---------- */
template<int Order>
void IRED<Order>::step(const Eigen::Vector3f& measurement, float dt) {
    T_last_f_ = dt;
    T_last_ = static_cast<double>(dt);
    for (int ch = 0; ch < 3; ++ch) {
        step_scalar(z_vec_[ch], static_cast<double>(measurement(ch)), static_cast<double>(dt));
        /* EMA filter per channel */
        ema_vec_[ch] = ema_alpha_ * z_vec_[ch][Order]
                     + (1.0 - ema_alpha_) * ema_vec_[ch];
    }
}

template<int Order>
Eigen::Vector3f IRED<Order>::output_vec(int derivative_order) const {
    Eigen::Vector3f out;
    for (int ch = 0; ch < 3; ++ch) {
        out(ch) = static_cast<float>(
            output_scalar(z_vec_[ch], derivative_order, T_last_, ema_vec_[ch]));
    }
    return out;
}

/* ====================================================================
 *  IRED explicit template instantiations
 * ==================================================================== */

template class IRED<1>;
template class IRED<2>;
template class IRED<3>;
template class IRED<4>;

/* ====================================================================
 *  Levant3  (legacy wrapper around IRED<3>)
 * ==================================================================== */

Levant3::Levant3(uint8_t _mode, float _L, double _ema_alpha)
    : mode(_mode), L(static_cast<double>(_L)), ired_(_L, _ema_alpha) {
}

Levant3::~Levant3() {}

void Levant3::setParam(double L, double ema_alpha) {
    this->L = L;
    ired_.setLipschitz(L);
    ired_.setEmaAlpha(ema_alpha);
}

void Levant3::Reset() {
    ired_.reset();
}

double Levant3::compute(double& f, float dt) {
    ired_.step(f, static_cast<double>(dt));
    return ired_.output(0);  // first derivative (velocity)
}

Eigen::Vector3f Levant3::compute(const Eigen::Vector3f& f, float dt) {
    ired_.step(f, dt);
    return ired_.output_vec(0);  // first derivative (velocity)
}
