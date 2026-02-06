#include "NMethods.h"
#include <Eigen/Dense>
#include <cmath>
#include <math.h>


float rk4(float(*fPtr)(float), const float iC, const float iCdt, const float dt){
    float a(0.0), b(0.0), c(0.0), d(0.0);
    a = dt * fPtr(iCdt);
    b = dt * fPtr(iCdt+a/2.0);
    c = dt * fPtr(iCdt+b/2.0);
    d = dt * fPtr(iCdt+c);
    return ((iC + (a+d)/6.0 + (b+c)/3.0));
}


Eigen::Vector3f rk4_vec(const Eigen::Vector3f iC, const Eigen::Vector3f iCdt, const float dt){
    Eigen::Vector3f integral(0,0,0);
    integral(0) = rk4(function1d, iC(0), iCdt(0), dt);
    integral(1) = rk4(function1d, iC(1), iCdt(1), dt);
    integral(2) = rk4(function1d, iC(2), iCdt(2), dt);
    return integral;
}


/*! \fn function1d.
 */
float function1d(float iCdt){
    return iCdt;
}

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

Eigen::Vector3f signth(const Eigen::Vector3f a, const float p){
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

void Levant_diff::setParam_vec(const Eigen::Vector3f alpha, const Eigen::Vector3f lamb){
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

float Levant_diff::Compute(const float f, const float dt){
    u = u1 - lamb*(sqrtf(fabs(x-f)))*sign_(x-f);

    u1p = -alpha*sign_(x-f);
    x = rk4(function1d, x, u, dt);
    u1 = rk4(function1d, u1, u1p, dt);

    err = x-f;
    return u;
}

void Levant_diff::Compute(float &_u, const float f, const float dt){
    _u = u1 - lamb*(sqrtf(fabs(x-f)))*sign_(x-f);

    u1p = -alpha*sign_(x-f);
    x = rk4(function1d, x, _u, dt);
    u1 = rk4(function1d, u1, u1p, dt);
}

Eigen::Vector3f Levant_diff::Compute(const Eigen::Vector3f f, const float dt){
    u_vec(0) = u1_vec(0) - lamb_vec(0)*(sqrtf(fabs(x_vec(0)-f(0))))*sign_(x_vec(0)-f(0));
    u_vec(1) = u1_vec(1) - lamb_vec(1)*(sqrtf(fabs(x_vec(1)-f(1))))*sign_(x_vec(1)-f(1));
    u_vec(2) = u1_vec(2) - lamb_vec(2)*(sqrtf(fabs(x_vec(2)-f(2))))*sign_(x_vec(2)-f(2));

    u1p_vec(0) = -alpha_vec(0)*sign_(x_vec(0)-f(0));
    u1p_vec(1) = -alpha_vec(1)*sign_(x_vec(1)-f(1));
    u1p_vec(2) = -alpha_vec(2)*sign_(x_vec(2)-f(2));

    x_vec = rk4_vec(x_vec, u_vec, dt);
    u1_vec = rk4_vec(u1_vec, u1p_vec, dt);

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
}

Levant3::~Levant3() {}

void Levant3::setParam(double L, double p) {
    this->L = L;
    this->p = p;
}

void Levant3::Reset() {
    z0 = 0.0F;
    z1 = 0.0F;
    z2 = 0.0F;
    z3 = 0.0F;

    z0_1 = Eigen::Vector3f::Zero();
    z1_1 = Eigen::Vector3f::Zero();
    z2_1 = Eigen::Vector3f::Zero();
    z3_1 = Eigen::Vector3f::Zero();
}

double Levant3::compute(double f, float dt) {
    double nu0 = (-50.0F*pow(L,1.0/4.0F)*pow(fabs(z0-f),3.0/4.0F)*sign_(z0-f)) + z1;
    double nu1 = (-20.0F*pow(L,1.0/3.0F)*pow(fabs(z1-nu0),2.0/3.0F)*sign_(z1-nu0)) + z2;
    double nu2 = (-8.0F*pow(L,1.0/2.0F)*pow(fabs(z2-nu1),1.0/2.0F)*sign_(z2-nu1)) + z3;
    double z3p = -1.1F*L*sign_(z3-nu2);

    z0 = rk4(function1d,z0,nu0,dt);
    z1 = rk4(function1d,z1,nu1,dt);
    z2 = rk4(function1d,z2,nu2,dt);
    z3 = rk4(function1d,z3,z3p,dt);

    //fp = nu0;
    //fp = nu0; // Compute the first derivative (velocity)

    return nu0;
}

Eigen::Vector3f Levant3::compute(const Eigen::Vector3f f, float dt) {
    Eigen::Vector3f nu0;
    Eigen::Vector3f nu1;
    Eigen::Vector3f nu2;
    Eigen::Vector3f z3p;

    nu0(0) = (-50.0F*pow(L,1.0/4.0F)*pow(fabs(z0_1(0)-f(0)),3.0/4.0F)*sign_(z0_1(0)-f(0))) + z1_1(0);
    nu0(1) = (-50.0F*pow(L,1.0/4.0F)*pow(fabs(z0_1(1)-f(1)),3.0/4.0F)*sign_(z0_1(1)-f(1))) + z1_1(1);
    nu0(2) = (-50.0F*pow(L,1.0/4.0F)*pow(fabs(z0_1(2)-f(2)),3.0/4.0F)*sign_(z0_1(2)-f(2))) + z1_1(2);

    nu1(0) = (-20.0F*pow(L,1.0/3.0F)*pow(fabs(z1_1(0)-nu0(0)),2.0/3.0F)*sign_(z1_1(0)-nu0(0))) + z2_1(0);
    nu1(1) = (-20.0F*pow(L,1.0/3.0F)*pow(fabs(z1_1(1)-nu0(1)),2.0/3.0F)*sign_(z1_1(1)-nu0(1))) + z2_1(1);
    nu1(2) = (-20.0F*pow(L,1.0/3.0F)*pow(fabs(z1_1(2)-nu0(2)),2.0/3.0F)*sign_(z1_1(2)-nu0(2))) + z2_1(2);

    nu2(0) = (-8.0F*pow(L,1.0/2.0F)*pow(fabs(z2_1(0)-nu1(0)),1.0/2.0F)*sign_(z2_1(0)-nu1(0))) + z3_1(0);
    nu2(1) = (-8.0F*pow(L,1.0/2.0F)*pow(fabs(z2_1(1)-nu1(1)),1.0/2.0F)*sign_(z2_1(1)-nu1(1))) + z3_1(1);
    nu2(2) = (-8.0F*pow(L,1.0/2.0F)*pow(fabs(z2_1(2)-nu1(2)),1.0/2.0F)*sign_(z2_1(2)-nu1(2))) + z3_1(2);

    z3p(0) = -1.1F*L*sign_(z3_1(0)-nu2(0)); 
    z3p(1) = -1.1F*L*sign_(z3_1(1)-nu2(1));
    z3p(2) = -1.1F*L*sign_(z3_1(2)-nu2(2));

    z0_1 = rk4_vec(z0_1,nu0,dt);
    z1_1 = rk4_vec(z1_1,nu1,dt);
    z2_1 = rk4_vec(z2_1,nu2,dt);
    z3_1 = rk4_vec(z3_1,z3p,dt);

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