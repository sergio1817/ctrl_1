#include "NMethods.h"
//#include <Eigen/Core>
#include <cmath>
//#include <math.h>
//#include <functional>
//#include <iostream>


float rk4o(float(*fPtr)(float), const float iC, const float iCdt, const float dt){
    float a(0.0), b(0.0), c(0.0), d(0.0);
    a = dt * fPtr(iCdt);
    b = dt * fPtr(iCdt+a/2.0);
    c = dt * fPtr(iCdt+b/2.0);
    d = dt * fPtr(iCdt+c);
    return ((iC + (a+d)/6.0 + (b+c)/3.0));
}

/*! \fn function1d.
 */
float function1d(float iCdt){
    return iCdt;
}

// RK4 implementations are templated in the header.

/*! \fn sign, función que contiene la función signo.
 */
float sign(const float a){
    if (a < 0)
        return -1;
    else if (a > 0)
        return 1;
    else
        return 0;
}


float sigmoide(const float a, const float d){
    float salida;
    float abs;

    if (a > 0)
        abs = a;
    else if (a < 0)
        abs = -a;
    else
        abs = 0;

    salida = (a/(abs+d));
    
    return salida;
}

float signth(const float a, const float p){
    return tanhf(p*a);
}

Eigen::Vector3f signth(const Eigen::Vector3f& a, const float p){
    Eigen::Vector3f salida;
    salida(0) = tanhf(p*a(0));
    salida(1) = tanhf(p*a(1));
    salida(2) = tanhf(p*a(2));
    return salida;
}

Levant_diff::Levant_diff(std::string _mode, const float _aplha, const float _lamb, const float _p): mode(_mode) {
    this->alpha = _aplha;
    this->lamb = _lamb;
    this->p = _p;
}

Levant_diff::~Levant_diff(){}

void Levant_diff::setParam(const float alpha, const float lamb, const float p){
    this->alpha = alpha;
    this->lamb = lamb;
    this->p = p;
}

void Levant_diff::setParam(const float alpha, const float lamb){
    this->alpha = alpha;
    this->lamb = lamb;
}

void Levant_diff::setParam_vec(const Eigen::Vector3f& alpha, const Eigen::Vector3f& lamb){
    this->alpha_vec = alpha;
    this->lamb_vec = lamb;
}

void Levant_diff::Reset(void){
    this->u = 0;
    this->u1 = 0;
    this->u1p = 0;
    this->x = 0;
    this->x_vec << 0, 0, 0;
    this->u_vec << 0, 0, 0;
    this->u1_vec << 0, 0, 0;
    this->u1p_vec << 0, 0, 0;
}

float Levant_diff::Compute(const float& f, const float dt){
    u = u1 - lamb*(sqrtf(fabs(x-f)))*sign_(x-f);

    u1p = -alpha*sign_(x-f);
    x = rk4(x, dt, u);
    u1 = rk4(u1, dt, u1p);

    err = x-f;
    return u;
}

void Levant_diff::Compute(float &_u, const float& f, const float dt){
    _u = u1 - lamb*(sqrtf(fabs(x-f)))*sign_(x-f);

    u1p = -alpha*sign_(x-f);
    x = rk4(x, dt, _u);
    u1 = rk4(u1, dt, u1p);
}

Eigen::Vector3f Levant_diff::Compute(const Eigen::Vector3f& f, const float dt){
    u_vec(0) = u1_vec(0) - lamb_vec(0)*(sqrtf(fabs(x_vec(0)-f(0))))*sign_(x_vec(0)-f(0));
    u_vec(1) = u1_vec(1) - lamb_vec(1)*(sqrtf(fabs(x_vec(1)-f(1))))*sign_(x_vec(1)-f(1));
    u_vec(2) = u1_vec(2) - lamb_vec(2)*(sqrtf(fabs(x_vec(2)-f(2))))*sign_(x_vec(2)-f(2));

    u1p_vec(0) = -alpha_vec(0)*sign_(x_vec(0)-f(0));
    u1p_vec(1) = -alpha_vec(1)*sign_(x_vec(1)-f(1));
    u1p_vec(2) = -alpha_vec(2)*sign_(x_vec(2)-f(2));

    x_vec = rk4_const(x_vec, dt, u_vec);
    u1_vec = rk4_const(u1_vec, dt, u1p_vec);

    err_v = x_vec-f;
    return u_vec;
}

float Levant_diff::getErr(void){
    return err;
}

Eigen::Vector3f Levant_diff::getErr_v(void){
    return err_v;
}

float Levant_diff::sign_(const float a){
    if(mode == "sign")
        return sign(a);
    else if(mode == "sigmoide")
        return 0;
    else if(mode == "tanh")
        return signth(a, p);
    else
        return sign(a);
}




Levant3::Levant3(uint8_t _mode, float _L, double _p): mode(_mode), L(_L), p(_p) {
    nu0 = Eigen::Vector3f::Zero();
    nu1 = Eigen::Vector3f::Zero();
    nu2 = Eigen::Vector3f::Zero();
    z3p = Eigen::Vector3f::Zero();
    updateLPowers();
}

Levant3::~Levant3() {}

void Levant3::setParam(double L, double p) {
    this->L = L;
    this->p = p;
    updateLPowers();
}

void Levant3::updateLPowers() {
    if (!std::isfinite(L) || L <= 0.0) {
        L_p14 = 0.0;
        L_p13 = 0.0;
        L_p12 = 0.0;
        return;
    }
    L_p14 = std::sqrt(std::sqrt(L));
    L_p13 = std::cbrt(L);
    L_p12 = std::sqrt(L);
}

void Levant3::Reset() {
    z0 = 0.0F;
    z1 = 0.0F;
    z2 = 0.0F;
    z3 = 0.0F;

    nu0 = Eigen::Vector3f::Zero();
    nu1 = Eigen::Vector3f::Zero();
    nu2 = Eigen::Vector3f::Zero();
    z3p = Eigen::Vector3f::Zero();

    z0_1 = Eigen::Vector3f::Zero();
    z1_1 = Eigen::Vector3f::Zero();
    z2_1 = Eigen::Vector3f::Zero();
    z3_1 = Eigen::Vector3f::Zero();
}

double Levant3::compute(double& f, float dt) {
    const double e0 = z0 - f;
    const double abs_e0 = std::fabs(e0);
    const double e0_3_4 = std::sqrt(std::sqrt(abs_e0 * abs_e0 * abs_e0));

    const double nu0_local = (-50.0 * L_p14 * e0_3_4 * sign_(e0)) + z1;
    const double e1_local = z1 - nu0_local;
    const double abs_e1 = std::fabs(e1_local);
    const double e1_2_3 = std::cbrt(abs_e1 * abs_e1);

    const double nu1_local = (-20.0 * L_p13 * e1_2_3 * sign_(e1_local)) + z2;
    const double e2_local = z2 - nu1_local;
    const double abs_e2 = std::fabs(e2_local);
    const double e2_1_2 = std::sqrt(abs_e2);

    const double nu2_local = (-8.0 * L_p12 * e2_1_2 * sign_(e2_local)) + z3;
    const double z3p_local = -1.1 * L * sign_(z3 - nu2_local);

    z0 = rk4_const(z0, static_cast<double>(dt), nu0_local);
    z1 = rk4_const(z1, static_cast<double>(dt), nu1_local);
    z2 = rk4_const(z2, static_cast<double>(dt), nu2_local);
    z3 = rk4_const(z3, static_cast<double>(dt), z3p_local);

    //fp = nu0;
    //fp = nu0; // Compute the first derivative (velocity)

    return nu0_local;
}

Eigen::Vector3f Levant3::compute(const Eigen::Vector3f& f, float dt) {
    for (int i = 0; i < 3; ++i) {
        const float e0 = z0_1(i) - f(i);
        const float abs_e0 = std::fabs(e0);
        const float e0_3_4 = std::sqrt(std::sqrt(abs_e0 * abs_e0 * abs_e0));

        nu0(i) = (-50.0F * static_cast<float>(L_p14) * e0_3_4 * static_cast<float>(sign_(e0))) + z1_1(i);

        const float e1 = z1_1(i) - nu0(i);
        const float abs_e1 = std::fabs(e1);
        const float e1_2_3 = std::cbrt(abs_e1 * abs_e1);

        nu1(i) = (-20.0F * static_cast<float>(L_p13) * e1_2_3 * static_cast<float>(sign_(e1))) + z2_1(i);

        const float e2 = z2_1(i) - nu1(i);
        const float abs_e2 = std::fabs(e2);
        const float e2_1_2 = std::sqrt(abs_e2);

        nu2(i) = (-8.0F * static_cast<float>(L_p12) * e2_1_2 * static_cast<float>(sign_(e2))) + z3_1(i);
        z3p(i) = -1.1F * static_cast<float>(L) * static_cast<float>(sign_(z3_1(i) - nu2(i)));
    }

    z0_1 = rk4_const(z0_1, dt, nu0);
    z1_1 = rk4_const(z1_1, dt, nu1);
    z2_1 = rk4_const(z2_1, dt, nu2);
    z3_1 = rk4_const(z3_1, dt, z3p);

    

    //std::cout << "Levant3 - err: " << (z0_1-f).transpose() << '\n';

    return nu0;
}

double Levant3::sign_(const double val){
    if(mode == 0){
        return sign(val);
    }
    if(mode == 1){
        return signth(val, p);
    }
    return sign(val);
}