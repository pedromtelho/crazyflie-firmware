#include "PwmOut.h"
#include "mbed.h"
#include "crazyflie.h"
#include "parameters.h"
#include "pin_names.h"

// Define motor 1 as PWM output object
PwmOut motor(MOTOR1);


// Converts desired angular velocity (rad/s) to PWM signal (%)
float control_motor(float omega_r)
{
    return a2*pow(omega_r, 2.0)+a1*omega_r;
}

 // Main program
 int main()
 {
     // Set PWM frequency to 500Hz
    motor.period(1.0/500.0);
    // Turn on motor 1 with 1.000 rad/s for 0.5s
    motor = control_motor(1000.0);
    wait (0.5);
    // Turn off motor 1
    motor = 0.0;
    // End of program
    while(true)
    {
    }
}