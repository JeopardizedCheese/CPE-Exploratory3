#pragma once
// ============================================================================
//  robot_ctrl settings for the course IN-ENG ESP32 board (ESP32-WROOM-32E).
//  Wheels: onboard TB6612 driven by the InEngMotor library, fixed pins
//          (left 26/27, right 16/17), PWM channels 0-3. Nothing to set here.
//  Other pins used on the board: servo header 19, TFT 18/23/5/2/4,
//  potentiometer 34, LED 23.
// ============================================================================

// ---------- Wheels ----------
#define L_INVERT true        // InEngMotor board default is (true, false); flip one if a wheel runs backwards
#define R_INVERT false
#define L_GAIN 1.00f         // straight-line trim: lower the stronger wheel (e.g. 0.90)
#define R_GAIN 1.00f
#define MAX_DUTY 1.00f       // 255 = full power
#define MIN_DUTY 0.71f       // ~200/255: below this the motors stall (measured). Any non-zero command starts here
#define RAMP_PER_SEC 3.0f    // speed-up limit (full scale per second); slow-down is instant

// ---------- Gripper servo (SG90) ----------
#define SERVO_PIN 19             // IN-ENG board servo header
#define SERVO_US_MIN 500         // pulse width at 0 deg   (same as course example 03)
#define SERVO_US_MAX 2500        // pulse width at 180 deg
#define SERVO_MIN_DEG 0          // never command outside this range
#define SERVO_MAX_DEG 55         // a little past closed
#define SERVO_START_DEG 0        // open at boot (the servo WILL jump here at power-on)
#define SERVO_DEG_PER_SEC 180.0f // slow moves reduce current spikes / brownout
#define GRIP_OPEN_DEG 0          // measured: jaws open
#define GRIP_CLOSE_DEG 45        // measured: holds every stone size

// ---------- Safety ----------
#define ESTOP_PIN 0              // BOOT button (press = stop). -1 disables
#define DRIVE_TIMEOUT_MS 300     // no drive packet for this long -> wheels stop
#define RUN_TIME_MS 300000UL     // 5 minutes from "start"
#define STATUS_LED 23            // board LED: off = IDLE, on = RUNNING, blinking = DONE/ESTOP. -1 = none

// ---------- Network ----------
#define UDP_PORT 4211
#define STATUS_PERIOD_MS 200
#define USE_STATIC_IP 1
#define STATIC_IP  10, 178, 188, 50
#define GATEWAY_IP 10, 178, 188, 223
#define SUBNET_IP  255, 255, 255, 0