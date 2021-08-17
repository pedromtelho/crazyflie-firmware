#include "mbed.h"
#include "crazyflie.h"
#include "mbed_wait_api.h"
#include "pin_names.h"

// Define all LEDs as digital output objects
DigitalOut led_verm(LED_RED_R,!false);
DigitalOut led_azul(LED_BLUE_L,!false);
DigitalOut led_verde_l(LED_GREEN_L,!false);
DigitalOut led_verde_r(LED_GREEN_R,!false);


// Define all motors as PWM objects
PwmOut motor_um(MOTOR1);
PwmOut motor_dois(MOTOR2);
PwmOut motor_tres(MOTOR3);
PwmOut motor_quatro(MOTOR4);

// Main program
int main()
{
    // Blink blue LED indicating inicialization (5 seconds)
    int initialization = 1;
    while(initialization) 
    {
        led_azul = !led_azul;
        wait(5.0);
        initialization = 0;
    }
    
    // Turn on red LEDs indicating motors are armed
    led_verm = 1;
    
    // Test all motors with different frequencies (to make different noises)
    motor_um = 0.1;
    wait(0.3);
    motor_um = 0.0;
    motor_dois = 0.4;
    wait(0.3);
    motor_dois = 0.0;
    motor_tres = 0.6;
    wait(0.3);
    motor_tres = 0.0;
    motor_quatro = 0.8;
    wait(0.3);
    motor_quatro = 0.0;

    
    // Turn off red LEDs indicating motors are disarmed
    led_verm = 0;
    
    // Blink green LEDs indicating end of program
    while(true)
    {
        led_verde_l = !led_verde_l;
        wait(0.1);
        led_verde_l = !led_verde_l;
        led_verde_r = !led_verde_r;
        wait(0.1);
        led_verde_r = !led_verde_r;
    }
}
