#ifndef CONFIG_H
#define CONFIG_H

// // Button
// #define BUTTON_PLAY_PIN 14
// #define BUTTON_STOP_PIN 16
// #define BUTTON_PAUSE_PIN 15
// #define DEBOUNCE_MS 500

// Bluetooth BLE
#define IAM "peripheral"
#define IAM_SENDING_TO "central"
#define BLE_NAME "peripheral"
#define BLE_SVC_UUID 1000
#define BLE_CHAR_P2C_UUID 0x2B6E
#define BLE_CHAR_C2P_UUID 0x2B5F
#define BLE_APPEARANCE 0x0300
#define BLE_ADVERTISING_INTERVAL 2000
#define BLE_ADVERTISING_INTERVAL 2000 //in milliseconds
#define BLE_SCAN_LENGTH 5000 // scan length in milliseconds
#define BLE_INTERVAL 30000 //interval in milliseconds
#define BLE_WINDOW 30000 //window in milliseconds

// Motor, encoder and gearbox
#define PIN_MOTOR_R_IN1 8
#define PIN_MOTOR_R_IN2 9
#define PIN_MOTOR_R_PWM 15
#define PIN_MOTOR_R_ENCODER_A 13
#define PIN_MOTOR_R_ENCODER_B 12

#define PIN_MOTOR_L_IN1 7
#define PIN_MOTOR_L_IN2 6
#define PIN_MOTOR_L_PWM 14
#define PIN_MOTOR_L_ENCODER_A 10
#define PIN_MOTOR_L_ENCODER_B 11

#define PWM_FREQ 1000
#define PWM_MAX 65535

#define ENCODER_RESOLUTION 7
#define REDUCER_RATIO 150

#define KP 10.0
#define KI 0.0
#define KD 0.1
#define SAMPLE_TIME_MS 20

// Robot physical parameters (ADDED)
#define WHEEL_RADIUS_M 0.017        // Wheel diameter in mm
#define WHEEL_BASE_M 0.075          // Distance between wheels in mm
#define MAX_RPM 60.0                 // Maximum RPM
#define RPM_CALC_INTERVAL 10.0      // RPM calculation interval in ms
#define DEFAULT_UPDATE_INTERVAL 10.0 // Default update interval in ms

#endif