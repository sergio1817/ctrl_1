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


Eigen::Vector3d rk4_vec(const Eigen::Vector3d iC, const Eigen::Vector3d iCdt, const float dt){
    Eigen::Vector3d integral(0,0,0);
    integral(0) = rk4(function1d, iC(0), iCdt(0), dt);
    integral(1) = rk4(function1d, iC(1), iCdt(1), dt);
    integral(2) = rk4(function1d, iC(2), iCdt(2), dt);
    return integral;
}

void Levants_diff(float &u, float &u1p, const float u1, const float x, const float f, const float alpha, const float lamb, const float p){
    u = u1 - lamb*(pow(abs(x-f),0.5))*signth(x-f,p);
    u1p = -alpha*signth(x-f,p);
}


// float Levants(const float f, const float alpha, const float lamb, const float p, const float dt){
//     float u, u1p;
//     float x = rk4(function1d, x, u, dt);
//     float u1 = rk4(function1d, u1, u1p, dt);
//     Levants_diff(u, u1p, u1, x, f, alpha, lamb, p);

//     return u;
// }

Eigen::Vector3d Levants_diff_vec(const Eigen::Vector3d f, const float alpha, const float lamb, const float p, const float dt){
    Eigen::Vector3d u(0,0,0), u1(0,0,0), u1p(0,0,0), x(0,0,0);

    u1 = rk4_vec(u1, u1p, dt);

    x = rk4_vec(x, u, dt);

    u(0) = u1(0) - lamb*(pow(abs(x(0)-f(0)),0.5))*signth(x(0)-f(0),p);
    u(1) = u1(1) - lamb*(pow(abs(x(1)-f(1)),0.5))*signth(x(1)-f(1),p);
    u(2) = u1(2) - lamb*(pow(abs(x(2)-f(2)),0.5))*signth(x(2)-f(2),p);

    u1p(0) = -alpha*signth(x(0)-f(0),p);
    u1p(1) = -alpha*signth(x(1)-f(1),p);
    u1p(2) = -alpha*signth(x(2)-f(2),p);

    

    return u;

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
    return tanh(p*a);
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
}

float Levant_diff::Compute(const float f, const float dt){
    u = u1 - lamb*(sqrtf(std::abs(x-f)))*sign_(x-f);

    u1p = -alpha*sign(x-f);
    x = rk4(function1d, x, u, dt);
    u1 = rk4(function1d, u1, u1p, dt);
    return u;
}

void Levant_diff::Compute(float &_u, const float f, const float dt){
    _u = u1 - lamb*(sqrtf(std::abs(x-f)))*sign_(x-f);

    u1p = -alpha*sign(x-f);
    x = rk4(function1d, x, _u, dt);
    u1 = rk4(function1d, u1, u1p, dt);
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