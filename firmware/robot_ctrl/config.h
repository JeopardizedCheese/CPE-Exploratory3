#pragma once
// ============================================================================
//  EDIT THIS FILE to match the real wiring. Every pin below is a placeholder.
//  ESP32 DevKit pins to AVOID: 6-11 (flash), 34-39 (input only),
//  0/2/12/15 (boot strapping), 1/3 (USB serial).
// ============================================================================

// ---------- Motor driver type ----------
// DRIVER_IN_IN_PWM : TB6612FNG, L298N  (IN1, IN2 direction + PWM/EN speed)
// DRIVER_TWO_PWM   : DRV8833, MX1508   (two PWM inputs per motor, *_PWM unused)
#define DRIVER_IN_IN_PWM 1
#define DRIVER_TWO_PWM   2
#define MOTOR_DRIVER DRIVER_IN_IN_PWM

#define L_IN1 26
#define L_IN2 27
#define L_PWM 25
#define R_IN1 32
#define R_IN2 33
#define R_PWM 13
#define MOTOR_STBY -1        // TB6612 STBY pin, or -1 if tied to 3.3V

#define L_INVERT false       // flip if "forward" spins this wheel backwards
#define R_INVERT false

#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_BITS 10
#define MAX_DUTY 0.60f       // start low; raise after first tests
#define MIN_DUTY 0.00f       // duty where wheels just start moving (measure it)
#define RAMP_PER_SEC 3.0f    // speed-up limit (full scale per second); slow-down is instant

// ---------- Servos (2 provided) ----------
#define SERVO_COUNT 2
#define SERVO_PINS      {18, 19}
#define SERVO_MIN_DEG   {0, 0}       // mechanical limits: measure so the arm never hits the frame
#define SERVO_MAX_DEG   {180, 180}
#define SERVO_START_DEG {90, 90}     // position at boot (servos WILL jump here at power-on)
#define SERVO_US_MIN 500
#define SERVO_US_MAX 2500
#define SERVO_DEG_PER_SEC 180.0f     // slow moves reduce current spikes / brownout

// Named poses (fill in after calibrating with the raw "servo" command)
#define GRIP_SERVO 0
#define GRIP_OPEN_DEG  60
#define GRIP_CLOSE_DEG 120
#define LIFT_SERVO 1
#define LIFT_UP_DEG    40
#define LIFT_DOWN_DEG  140

// ---------- Safety ----------
#define ESTOP_PIN 23             // push button to GND (INPUT_PULLUP). -1 disables: NOT recommended
#define DRIVE_TIMEOUT_MS 300     // no drive packet for this long -> wheels stop
#define RUN_TIME_MS 300000UL     // 5 minutes from "start"
#define UDP_PORT 4211
#define STATUS_PERIOD_MS 200
#define STATUS_LED 2
