#ifndef parameters_h
#define parameters_h

#include <cmath>

const float dt_range = 0.05;

// Physical constants
const float pi = 3.1416;
const float g = 9.81;       // m/s^2

const float a2 = 1.16e-7; // (Ra * kd)/(Km * es)
const float a1 = 4.488e-12; // (Ra * b + Km^2)/(Km * es)
const float kl = 1.726e-8; // constante de sustentação
const float kd = 1.415e-10; // constante de arrasto

// Quadcopter dimensions
const float m = 30.0e-3;    // kg
const float I_xx = 16.0e-6; // kg.m^2
const float I_yy = 16.0e-6; // kg.m^2
const float I_zz = 29.0e-6; // kg.m^2
const float l = 33.0e-3;    // m
const float dt = 1.0/500.0;
const float omega_c_att = 1.0;


const float Ts_phi = 0.3;
const float OS_phi = 0.005;
const float zeta_phi = abs(log(OS_phi))/sqrt(pow(log(OS_phi),2)+pow(pi,2));
const float omega_n_phi = 4.0/(Ts_phi*zeta_phi);
const float kp_phi = pow(omega_n_phi,2.0);       
const float kd_phi = 2.0*zeta_phi*omega_n_phi; 
const float kp_theta = kp_phi;       
const float kd_theta = kd_phi;

const float Ts_psi = 0.6;
const float OS_psi = 0.005;
const float zeta_psi = abs(log(OS_psi))/sqrt(pow(log(OS_psi),2)+pow(pi,2));
const float omega_n_psi = 4.0/(Ts_psi*zeta_psi);
const float kd_psi = pow(omega_n_psi,2.0);       
const float kp_psi = 2.0*zeta_psi*omega_n_psi; 


// Vertical estimator
const float omega_c_ver = 10.0;         // rad/s  
const float zeta_ver = sqrt(2.0)/2.0; 
const float ld_ver = pow(omega_c_ver,2);
const float lp_ver = 2.0*zeta_ver*omega_c_ver;


// Vertical controller
const float Ts_z = 2.0;                 // s
const float OS_z = 0.005;               // %
const float zeta_z = abs(log(OS_z))/sqrt(pow(log(OS_z),2)+pow(pi,2));
const float omega_n_z = 4.0/(Ts_z*zeta_z);
const float kp_z = pow(omega_n_z,2.0);       
const float kd_z = 2.0*zeta_z*omega_n_z;

// horizontal estimator
const float gamma = 42.0;               // º
const float resolution = 420.0;         // px
const float sigma = (1.0/dt)*(2.0*tan((gamma*pi/180.0)/2.0))/resolution; 

const float omega_c_hor = 50.0;         // rad/s 

// horizontal controller
const float Ts_x = 2.0;                 // s
const float OS_x = 0.005;               // %
const float zeta_x = abs(log(OS_x))/sqrt(pow(log(OS_x),2)+pow(pi,2));
const float omega_n_x = 4.0/(Ts_x*zeta_x);
const float kp_x = pow(omega_n_x,2.0);       
const float kd_x = 2.0*zeta_x*omega_n_x; 
const float kp_y = kp_x;       
const float kd_y = kd_x; 

#endif