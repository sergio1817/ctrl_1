#include "Rk4.h"
#include <math.h>

/*! \fn rk4, método de integración (Runge-Kutta de 4to orden).
 */
float rk4(float(*fPtr)(float), const float iC, const float iCdt, const float dt){
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
    float salida;

    salida= tanh(p*a);
    
    return salida;
}
