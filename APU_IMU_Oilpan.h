#pragma once

/*
 * Adapted from ArduCopter libraries for use with the Arduino Uno
 * by Eric Paniagua
 *
 */

#include <AP_Math.h>
#include "APU_Common.h"
#include "IMU.h"
#include "board.h"

class APU_IMU_Oilpan : public IMU
{
public:
    /* Constructor
     *
     * Saves the ADC pointer
     */
    APU_IMU_Oilpan (APU_ADC *adc);

    /* Do warm or cold start.
     *
     * NOTE: Warm start not implemented! Cold start always forced.
     *
     * NOTE:  For a partial-warmstart where e.g. the accelerometer calibration should be preserved
     *        but the gyro cal needs to be re-performed, start with ::init(WARM_START) to load the
     *        previous calibration settings, then force a re-calibration of the gyro with ::init_gyro.
     *
     * style  Selects the initialization style.
     *        COLD_START performs calibration of both the accelerometer and gyro.
     *        WARM_START loads accelerometer and gyro calibration from a previous cold start.
     */
    virtual void init (Start_style style = COLD_START);

    virtual void save (); // does nothing - no persistence
    virtual void init_accel ();
    virtual void init_gyro ();
    virtual bool update ();

private:
    APU_ADC             *_adc;          // ADC that we use for reading sensors
//    AP_VarA<float,5>    _sensor_cal;    // Calibrated sensor offsets
    float               _sensor_cal[5];
    // Switched away from AP_VarA - this is the EEPROM functionality

    virtual void        _init_accel (); // no-save implementation
    virtual void        _init_gyro ();  // no-save implementation

    float               _sensor_compensation (uint8_t channel, int temp) const;
    float               _sensor_in (uint8_t channel, int temperature);

    // constants
    static const uint8_t    _sensors[5];            // ADC channel mappings for the sensors
    static const int8_t     _sensor_signs[5];       // ADC result sign adjustment for sensors
    static const uint8_t    _gyro_temp_ch = GYRO_TEMP_CHANNEL;
    static const float      _gyro_temp_curve[3][3]; // Temperature compensation curve for the gyro

    // ADC : Voltage reference 3.3v / 10bits(1024 steps) => 3.2mV/ADC step
    // ADXL335 Sensitivity(from datasheet) => 330mV/g, 3.2mV/ADC step => 330/3.2 = 103 ADC steps/g
    // Tested value : ???
#define GRAVITY_CONSTANT 423.8;
    static const float      _gravity = GRAVITY_CONSTANT;       // <1G in the raw data coming from the accelerometer
                                                    // Value based on actual sample data from 20 boards
                                                    // (Taken directly from arducopter source.)
    static const float      _accel_scale = 9.80665 / GRAVITY_CONSTANT; // would like to use _gravity here, but cannot

    // IDG500 Sensitivity (from datasheet) => 2.0mV/degree/s, 3.2mV/ADC step => 1.6 degree/s / ADC step
    // Tested values : ???
    //
    static const float      _gyro_gain_x = 0.4;     // X axis Gyro gain
    static const float      _gyro_gain_y = 0.41;    // Y axis Gyro gain
    static const float      _gyro_gain_z = 0.41;    // Z axis Gyro gain

    // Maximum possible value returned by an offset-corrected sensor-channel
    //
    static const float      _adc_constraint = 900;

    // Gyro and Accelerometer calibration criteria
    //
    static const float      _gyro_total_cal_change = 4.0;       // Experimentally derived - allows for some minor motion
    static const float      _gyro_max_cal_offset = 320.0;
    static const float      _accel_total_cal_change = 4.0;
    static const float      _accel_max_cal_offset = 250.0;
};
