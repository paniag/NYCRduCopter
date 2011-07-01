#pragma once

/*
 * Adapted from ArduCopter libraries for use with the Arduino Uno
 * by Eric Paniagua
 *
 */

#include <inttypes.h>
#include <AP_Math.h>
#include "APU_ADC.h"

class IMU
{
public:
    // Constructor
    IMU() {}

    enum Start_style {
        COLD_START = 0,
        WARM_START
    };

    /* Perform startup initialization.
     *
     * Called to initialize the state of the IMU.
     *
     * For COLD_START, implementations using real sensors can assume
     * that the airframe is stationary and nominally oriented.
     *
     * For WARM_START, no assumptions should be made about the
     * orientation or motion of the airframe.  Calibration should be
     * as for the previous COLD_START call.
     */
    virtual void    init(Start_style style) = 0;

    // Perform cold startup initialization for just the accelerometers.
    virtual void    init_accel() = 0;

    // Perform cold startup initialization for just the gyros.
    virtual void    init_gyro() = 0;

    /* Give the IMU some cycles to perform/fetch an update from its
     * sensors.
     */
    virtual bool    update() = 0;

    // Fetch the current accelerometer values
    Vector3f        get_accel() { return _accel; }

    // Fetch the current gyro values
    Vector3f        get_gyro() { return _gyro; }

    // A count of the number of times sensor readings were considered out of bounds
    uint8_t         adc_constraints;

protected:
    // Most recent accelerometer reading obtained by ::update
    Vector3f        _accel;

    // Most recent gyro reading obtained by ::update
    Vector3f        _gyro;
};
