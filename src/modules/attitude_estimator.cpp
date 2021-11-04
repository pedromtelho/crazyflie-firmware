#include "attitude_estimator.h"

// Class constructor
AttitudeEstimator::AttitudeEstimator() : imu(IMU_SDA, IMU_SCL)
{
    phi = 0.0;
    theta = 0.0;
    psi = 0.0;
    p = 0.0;
    q = 0.0;
    r = 0.0;
    p_bias = 0.0;
    q_bias = 0.0;
    r_bias = 0.0;
}

// Initialize class
void AttitudeEstimator::init()
{
    imu.init();
    for (int i = 0; i < 500; i++) {
        imu.read();
        p_bias += imu.gx/500.0;
        q_bias += imu.gy/500.0;
        r_bias += imu.gz/500.0;
        wait(dt);
    }
}

// Estimate Euler angles (rad) and angular velocities (rad/s)
void AttitudeEstimator::estimate()
{
    imu.read();
    float phi_a = atan2(-imu.ay,-imu.az);
    float theta_a = atan2(imu.ax,-((imu.az>0)-(imu.az<0))*sqrt(pow(imu.ay,2)+pow(imu.az,2)));
    p = imu.gx - p_bias;
    q = imu.gy - q_bias;
    r = imu.gz - r_bias;
    

    float phi_g = phi + (p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta))*dt;
    float theta_g = theta + (q*cos(phi)-r*sin(phi))*dt;
    float psi_g = psi + (q*sin(phi)/cos(theta)+r*cos(phi)/cos(theta))*dt;
    
    phi = (1.0-(omega_c_att*dt))*phi_g+(omega_c_att*dt)*phi_a;
    theta = (1.0-(omega_c_att*dt))*theta_g+(omega_c_att*dt)*theta_a;
    psi = psi_g;
}