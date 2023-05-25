#include "NMethods.h"
#include <Eigen/Dense>
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
    return u;
}

void Levant_diff::Compute(float &_u, const float f, const float dt){
    _u = u1 - lamb*(sqrtf(fabs(x-f)))*sign_(x-f);

    u1p = -alpha*sign_(x-f);
    x = rk4(function1d, x, _u, dt);
    u1 = rk4(function1d, u1, u1p, dt);
}

Eigen::Vector3f Levant_diff::Compute(const Eigen::Vector3f f, const float dt){
    u_vec(0) = u1_vec(0) - lamb*(sqrtf(fabs(x_vec(0)-f(0))))*sign_(x_vec(0)-f(0));
    u_vec(1) = u1_vec(1) - lamb*(sqrtf(fabs(x_vec(1)-f(1))))*sign_(x_vec(1)-f(1));
    u_vec(2) = u1_vec(2) - lamb*(sqrtf(fabs(x_vec(2)-f(2))))*sign_(x_vec(2)-f(2));

    u1p_vec(0) = -alpha*sign_(x_vec(0)-f(0));
    u1p_vec(1) = -alpha*sign_(x_vec(1)-f(1));
    u1p_vec(2) = -alpha*sign_(x_vec(2)-f(2));

    x_vec = rk4_vec(x_vec, u_vec, dt);
    u1_vec = rk4_vec(u1_vec, u1p_vec, dt);
    return u_vec;
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