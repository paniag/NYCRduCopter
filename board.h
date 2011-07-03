#pragma once

/*
 * Digital pin mappings
 */

// LED pin mappings
#define A_LED_PIN         13
#define C_LED_PIN         5

/*
 * ADC pin mappings
 *
 * Connect AREF to 3.3V
 */

// IMU sensor channels
#define ACCEL_X_CHANNEL   0
#define ACCEL_Y_CHANNEL   1
#define ACCEL_Z_CHANNEL   2
#define GYRO_X_CHANNEL    3
#define GYRO_Y_CHANNEL    4
#define GYRO_TEMP_CHANNEL 5

// Sensor indices - DO NOT CHANGE
#define ACCEL_X           0
#define ACCEL_Y           1
#define ACCEL_Z           2
#define GYRO_X            3
#define GYRO_Y            4

#define ACCEL_START       ACCEL_X
#define ACCEL_END         ACCEL_Z
#define GYRO_START        GYRO_X
#define GYRO_END          GYRO_Y
