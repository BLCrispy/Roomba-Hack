#include <cstdio>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// UART configuration
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Define the GPIO pins connected to the L298N
const uint PIN_IN1 = 2;
const uint PIN_IN2 = 3;
const uint PIN_IN3 = 4;
const uint PIN_IN4 = 5;
const uint PIN_ENA = 6; // Must be a PWM capable pin (for speed enable A)
const uint PIN_ENB = 7; // Must be a PWM capable pin (for speed enable B)

// Servo Motor Pin Configuration
#define SERVO1_PIN 8   // PWM slice 7 channel A
#define SERVO2_PIN 9   // PWM slice 7 channel B
#define DEADZONE 8

// Servo PWM
#define SERVO_MIN_US 600
#define SERVO_MAX_US 2400
#define SERVO_PERIOD_US 20000   // 20ms (50Hz)

// Servo PWM Slice and channel init
uint servo1_slice, servo1_channel;
uint servo2_slice, servo2_channel;

float servo1_angle = 90;
float servo2_angle = 90;

const uint LASER_PIN = 10; // Initialize Laser pin
static uint16_t lastButtons = 0; // For button press logic

// Must match sender packet exactly
struct __attribute__((packed)) ControllerPacket {
    uint16_t buttons;
    uint8_t lx;
    uint8_t ly;
    uint8_t rx;
    uint8_t ry;
    uint8_t l2;
    uint8_t r2;
    uint8_t hat;
};



int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int constrain(int x, int a, int b) {
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

void setup_servo(uint pin, uint *slice, uint *channel)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);

    *slice = pwm_gpio_to_slice_num(pin);
    *channel = pwm_gpio_to_channel(pin);

    pwm_config config = pwm_get_default_config();

    pwm_config_set_clkdiv(&config, 64.0f);
    pwm_config_set_wrap(&config, 39062);

    pwm_init(*slice, &config, true);
}

void setup_motor_pwm(uint gpio_pin) {
    // Set the GPIO function to PWM
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);
    
    // Find out which PWM slice is connected to the given pin
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    uint channel = pwm_gpio_to_channel(gpio_pin); // Get the channel for the pin

    // Set the PWM wrap value (frequency is determined by wrap and clock divider)
    pwm_set_wrap(slice_num, 65535); // Max value for 16-bit duty cycle

    // Set the initial duty cycle to 0 (motor off)
    pwm_set_chan_level(slice_num, channel, 0);

    // Enable the PWM slice
    pwm_set_enabled(slice_num, true);
}

void setup_motor_pins() {
    // Initialize direction pins as outputs
    gpio_init(PIN_IN1);
    gpio_set_dir(PIN_IN1, GPIO_OUT);
    gpio_init(PIN_IN2);
    gpio_set_dir(PIN_IN2, GPIO_OUT);
    gpio_init(PIN_IN3);
    gpio_set_dir(PIN_IN3, GPIO_OUT);
    gpio_init(PIN_IN4);
    gpio_set_dir(PIN_IN4, GPIO_OUT);

    // Setup ENA pin for PWM
    setup_motor_pwm(PIN_ENA);
    setup_motor_pwm(PIN_ENB);
}

void stop_motor() {
    // All Motor pins set to 0 for stopping
    gpio_put(PIN_IN1, 0);
    gpio_put(PIN_IN2, 0);
    gpio_put(PIN_IN3, 0);
    gpio_put(PIN_IN4, 0);
    // Set speed to 0
    pwm_set_gpio_level(PIN_ENA, 0);
    pwm_set_gpio_level(PIN_ENB, 0);
} 

void motor_move(uint16_t turningInput, uint16_t r2, uint16_t l2) {
    // Initialize speed
    int speed;

    // Define deadzone for turning input
    int deadzoneMin = 123;
    int deadzoneMax = 133;

    if ((r2-l2) > 0) { // If moving forward sets values
        // Set Motors to both go forward
        gpio_put(PIN_IN1, 1);
        gpio_put(PIN_IN2, 0);
        gpio_put(PIN_IN3, 1);
        gpio_put(PIN_IN4, 0);

        // Set forward speed to difference of speed triggers
        speed = r2-l2;
    }
    else if ((r2-l2) < 0) { // If moving backwards sets values
        // Set Motors to both go backwards
        gpio_put(PIN_IN1, 0);
        gpio_put(PIN_IN2, 1);
        gpio_put(PIN_IN3, 0);
        gpio_put(PIN_IN4, 1);

        // Set backwards speed to diffference of speed triggers 
        speed = l2-r2;
    }
    else { // If no movement stops motors
        stop_motor();
        return; // Early break out of function since no need to read rest of function
    }

    // Sets each base motor speed at whatever the difference of rx and lx was set to
    int leftSpeed = speed;
    int rightSpeed = speed;

    // Set the speed (duty cycle)
    if (turningInput < deadzoneMin) // Left Turn (5 int deadzone)
    {
        // Turning left (input value is lower than center)
        int turnAmount = map(turningInput, 0, deadzoneMin - 1, 0, 255);
        leftSpeed = map(turnAmount, 0, 255, 0, leftSpeed); // Reduce left speed
        //rightSpeed = map(turnAmount, 0, 255, 0, rightSpeed); // Keep right speed high

        // Confine speed values to the valid range (0-255)
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);
    }
    else if (turningInput > deadzoneMax) // Right Turn (5 int deadzone)
    { 
        // Turning right (input value is higher than center)
        int turnAmount = map(turningInput, deadzoneMax + 1, 255, 255, 0);
        rightSpeed = map(turnAmount, 0, 255, 0, rightSpeed); // Reduce right speed
        //leftSpeed = map(turnAmount, 0, 255, 0, leftSpeed); // Keep left speed high

        // Confine speed values to the valid range (0-255)
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);
    }
    // Runs motors forward at speed given if in deadzone otherwise changed by if block above
    pwm_set_gpio_level(PIN_ENA, leftSpeed*257);
    pwm_set_gpio_level(PIN_ENB, rightSpeed*257);
    // speeds multiplied by 257 to put speed in reference to PWM Wrapper that spans 0-65535
    //printf("Left Speed: %d Right Speed: %d lx: %d l2: %d r2: %d\n", leftSpeed, rightSpeed, turningInput, l2, r2);
    //fflush(stdout);
}

void servo_set_angle(uint slice, uint channel, float angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint16_t pulse = SERVO_MIN_US +
        (angle * (SERVO_MAX_US - SERVO_MIN_US) / 180.0f);

    uint16_t level = pulse * 39062 / SERVO_PERIOD_US;

    pwm_set_chan_level(slice, channel, level);
}

int apply_deadzone(int value)
{
    if (abs(value - 128) < DEADZONE)
        return 128;

    return value;
}

float smooth_move(float current, float target, float speed)
{
    return current + (target - current) * speed;
}



int main() {
    stdio_init_all();

    gpio_init(LASER_PIN);
    gpio_set_dir(LASER_PIN, GPIO_OUT);
    gpio_put(LASER_PIN, 0);

    // toggle for laser
    int laserToggle = 0;

    // UART init
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Servo setup
    setup_servo(SERVO1_PIN, &servo1_slice, &servo1_channel);
    setup_servo(SERVO2_PIN, &servo2_slice, &servo2_channel);

    // DC Motor setup
    setup_motor_pins();

    sleep_ms(1000);
    printf("Receiver Pico ready\n");

    ControllerPacket packet;

    uint8_t start;

    while (true) {

        do {
            uart_read_blocking(UART_ID, &start, 1);
        } while (start != 0xAA);

        // Blocking read for full packet
        uart_read_blocking(UART_ID,
            reinterpret_cast<uint8_t*>(&packet),
            sizeof(packet));

        // Print received data
        printf("Buttons: %04x | LX:%d LY:%d RX:%d RY:%d L2:%d R2:%d Hat:%d\n",
               packet.buttons,
               packet.lx, packet.ly,
               packet.rx, packet.ry,
               packet.l2, packet.r2,
               packet.hat);
        
        // Take ds4 left joystick and r/l trigger inputs to control DC motor movement
        motor_move(packet.lx, packet.r2, packet.l2);

        // Apply deadzone
        packet.rx = apply_deadzone(packet.rx);
        packet.ry = apply_deadzone(packet.ry);

        // Convert joystick to target angle
        float targetX = (255 - packet.rx) * 180.0f / 255.0f;
        float targetY = packet.ry * 180.0f / 255.0f;

        // Smooth motion
        servo1_angle = smooth_move(servo1_angle, targetX, 1.0f);
        servo2_angle = smooth_move(servo2_angle, targetY, 1.0f);

        // Only update servo if movement is significant
        static float last_sent_x = 90;
        static float last_sent_y = 90;

        if (fabs(servo1_angle - last_sent_x) > 1.5f) {
            servo_set_angle(servo1_slice, servo1_channel, servo1_angle);
            last_sent_x = servo1_angle;
        }

        if (fabs(servo2_angle - last_sent_y) > 1.5f) {
            servo_set_angle(servo2_slice, servo2_channel, servo2_angle);
            last_sent_y = servo2_angle;
        }

        // Move servos
        //servo_set_angle(servo1_slice, servo1_channel, servo1_angle);
        //servo_set_angle(servo2_slice, servo2_channel, servo2_angle);


        // Checks if x has been pressed and encompasses other combos that include x, if true toggles laser on
        if ((packet.buttons & 0x2000) && !(lastButtons & 0x2000)) {
            if (laserToggle == 0) {
                gpio_put(LASER_PIN, 1);
                laserToggle = 1;
            }
            else {
                gpio_put(LASER_PIN, 0);
                laserToggle = 0;
            }
        }   

        lastButtons = packet.buttons;

        sleep_ms(20);
    }
}