#ifndef RK4_H
#define RK4_H


/*! \fn rk4, método de integración (Runge-Kutta de 4to orden).
 */
float rk4(float(*fPtr)(float), const float iC, const float iCdt, const float dt);

/*! \fn function1d.
 */
float function1d(float iCdt);

/*! \fn sign, función que contiene la función signo.
 */
float sign(const float a);

float sigmoide(const float a, const float d);

float signth(const float a, const float p);

#endif // RK4_H
