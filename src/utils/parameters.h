#ifndef parameters_h
#define parameters_h

#include <cmath>

// Physical constants
const float pi = 3.1416;
const float g = 9.81;       // m/s^2

const float a2 = pow(1.16, -7.0); // (Ra * kd)/(Km * es)
const float a1 = pow(4.488, -12.0); // (Ra * b + Km^2)/(Km * es)
const float kt = pow(1.726, -8.0); // constante de sustentação
// Quadcopter dimensions
const float m = 30.0e-3;    // kg
const float I_xx = 16.0e-6; // kg.m^2
const float I_yy = 16.0e-6; // kg.m^2
const float I_zz = 29.0e-6; // kg.m^2
const float l = 33.0e-3;    // m

#endif