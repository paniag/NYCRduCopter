#include "APU_IMU_Oilpan.h"

#include "board.h"
#include "calibration.h"

// Sensors: ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y
// Channel assignments on the APU oilpan
const uint8_t APU_IMU_Oilpan::_sensors[5] = {
    ACCEL_X_CHANNEL,
    ACCEL_Y_CHANNEL,
    ACCEL_Z_CHANNEL,
    GYRO_X_CHANNEL,
    GYRO_Y_CHANNEL
};
// Channel orientation vs. normal
const int8_t  APU_IMU_Oilpan::_sensor_signs[5] = { 1,-1,-1, 1,-1 };   // Channel orientation vs. normal

// Temp compensation curve constants
// These must be produced by measuring data and curve fitting
// [X/Y/Z gyro][A/B/C or 0 order/1st order/2nd order constants]
//
const float APU_IMU_Oilpan::_gyro_temp_curve[3][3] = {
    {1658,0,0},           // Values to use if no temp compensation data available
    {1658,0,0},           // Based on average values for 20 sample boards
    {1658,0,0}            // (Copied directly from arducopter.)
};

APU_IMU_Oilpan::APU_IMU_Oilpan (APU_ADC *adc) :
  _adc(adc)
{}

void APU_IMU_Oilpan::init (Start_style style) {
    // force a cold start
    style = COLD_START;

    // if we are warm-starting, load the calibration data from EEPROM and go
    //
    if (style == WARM_START) {
//        _sensor_cal.load();
    } else {
        // do cold-start calibration for both accel and gyro
        _init_gyro();
        _init_accel();

        // save calibration
//        _sensor_cal.save();
    }
}

void APU_IMU_Oilpan::init_gyro () {
    _init_gyro();
//    _sensor_cal.save();
}

void APU_IMU_Oilpan::_init_gyro() {
    int flashcount = 0;
    int tc_temp;
    float adc_in;
    float prev[3] = {0,0,0};
    float total_change;
    float max_offset;

    // cold start
    tc_temp = _adc->Ch(GYRO_TEMP_CHANNEL);
    delay(500);
    //Serial.printf_P(PSTR("Init Gyro"));
    Serial.println("Init Gyro");

    // Mostly we are just flashing the LEDs here to tell the user to keep the IMU still
    for (int c = 0; c < 25; c++) {
        digitalWrite(A_LED_PIN, LOW);
        digitalWrite(C_LED_PIN, HIGH);
        delay(20);

        for (int i = 0; i < 5; i++)
            adc_in = _adc->Ch(_sensors[i]);

        digitalWrite(A_LED_PIN, HIGH);
        digitalWrite(C_LED_PIN, LOW);
        delay(20);
    }

    for (int j = GYRO_START; j <= GYRO_END; j++)
        _sensor_cal[j] = 500;       // Just a large value to load prev[j] the first time

    do {
        for (int j = GYRO_START; j <= GYRO_END; j++) {
            prev[j]         = _sensor_cal[j];
            adc_in          = _adc->Ch(_sensors[j]);
            adc_in         -= _sensor_compensation(j, tc_temp);
            _sensor_cal[j]  = adc_in;
        }

        for (int i = 0; i < 50; i++) {
            for (int j = GYRO_START; j <= GYRO_END; j++) {
                adc_in          = _adc->Ch(_sensors[j]);
                // Subtract temp compensated typical gyro bias
                adc_in         -= _sensor_compensation(j, tc_temp);
                // filter
                _sensor_cal[j]  = _sensor_cal[j] * 0.9 + adc_in * 0.1;
            }

            delay(20);
            if (flashcount == 5) {
                //Serial.printf_P(PSTR("*"));
                Serial.print("*");
                digitalWrite(A_LED_PIN, LOW);
                digitalWrite(C_LED_PIN, HIGH);
            }

            if (flashcount >= 10) {
                flashcount = 0;
                digitalWrite(A_LED_PIN, HIGH);
                digitalWrite(C_LED_PIN, LOW);
            }
            flashcount++;
        }

        total_change = 0.0;
        for (int j = GYRO_START; j <= GYRO_END; j++)
            total_change += fabs(prev[j] - _sensor_cal[j]);

        max_offset = _sensor_cal[GYRO_START];
        for (int j = GYRO_START+1; j <= GYRO_END; j++)
            max_offset = (max_offset > _sensor_cal[j]) ? max_offset : _sensor_cal[j];

        delay(500);
    } while (total_change > _gyro_total_cal_change || max_offset > _gyro_max_cal_offset);
}

void APU_IMU_Oilpan::save () {
//    _sensor_cal.save();
}

void APU_IMU_Oilpan::init_accel () {
    _init_accel();
}

void APU_IMU_Oilpan::_init_accel () {
    int flashcount = 0;
    float adc_in;
    float prev[5] = {0,0,0};
    float total_change;
    float max_offset;

    // cold start
    delay(500);

    //Serial.printf_P(PSTR("Init Accel"));
    Serial.println("Init Accel");

    for (int j = ACCEL_START; j <= ACCEL_END; j++)
        _sensor_cal[j] = 500;     // Just a large value to load prev[j] the first time

    do {
        for (int j = ACCEL_START; j <= ACCEL_END; j++) {
            prev[j]         = _sensor_cal[j];
            adc_in          = _adc->Ch(_sensors[j]);
            adc_in         -= _sensor_compensation(j, 0); // temperature ignored
            _sensor_cal[j]  = adc_in;
        }

        for (int i = 0; i < 50; i++) {          // We take some readings...
            delay(20);

            for (int j = ACCEL_START; j <= ACCEL_END; j++) {
                adc_in          = _adc->Ch(_sensors[j]);
                adc_in         -= _sensor_compensation(j, 0);   // temperature ignored
                _sensor_cal[j]  = _sensor_cal[j] * 0.9 + adc_in * 0.1;
            }

            if (flashcount == 5) {
                //Serial.printf_P(PSTR("*"));
                Serial.print("*");
                digitalWrite(A_LED_PIN, LOW);
                digitalWrite(C_LED_PIN, HIGH);
            }

            if (flashcount >= 10) {
                flashcount = 0;
                digitalWrite(A_LED_PIN, HIGH);
                digitalWrite(C_LED_PIN, LOW);
            }
            flashcount++;
        }

        // null gravity from the Z accel
        _sensor_cal[ACCEL_Z] += _gravity * _sensor_signs[ACCEL_Z];

        total_change = 0.0;
        for (int j = ACCEL_START; j <= ACCEL_END; j++)
            total_change += fabs(prev[j] - _sensor_cal[j]);

        max_offset = _sensor_cal[ACCEL_START];
        for (int j = ACCEL_START+1; j <= ACCEL_END; j++)
            max_offset = (max_offset > _sensor_cal[j]) ? max_offset : _sensor_cal[j];

        delay(500);
    } while (total_change > _accel_total_cal_change || max_offset > _accel_max_cal_offset);

    //Serial.printf_P(PSTR(" "));
    Serial.print(" ");
}

/*
 * Returns the temperature compensated raw gyro value
 */
float APU_IMU_Oilpan::_sensor_compensation (uint8_t channel, int temperature) const {
    switch (channel) {
        case ACCEL_X:
        case ACCEL_Y:
        case ACCEL_Z:
            // do fixed-offset accelerometer compensation
            return AVG_RAW_ADC_ACCEL_ZERO;
        case GYRO_X:
        case GYRO_Y:
            // do gyro temperature compensation
            return  _gyro_temp_curve[channel - GYRO_START][0] +
                    _gyro_temp_curve[channel - GYRO_START][1] * temperature +
                    _gyro_temp_curve[channel - GYRO_START][2] * temperature * temperature;
        default:
            return 0; // no compensation
    }
}

/*
 * Returns a sensor value corrected for temperature and calibration offsets
 */
float APU_IMU_Oilpan::_sensor_in (uint8_t channel, int temperature)
{
    float adc_in;

    // get the compensated sensor value
    //
    adc_in = _adc->Ch(_sensors[channel]) - _sensor_compensation(channel, temperature);

    // adjust for sensor sign and apply calibration offset
    //
    if (_sensor_signs[channel] < 0) {
        adc_in = _sensor_cal[channel] - adc_in;
    } else {
        adc_in = adc_in - _sensor_cal[channel];
    }

    // constrain sensor readings to the sensible range
    //
    if (fabs(adc_in) > _adc_constraint) {
        adc_constraints++;                                              // We keep track of the number of times
        adc_in = constrain(adc_in, -_adc_constraint, _adc_constraint);  // Throw out nonsensical values
    }

    return adc_in;
}

/*
 * Populates the _gyro and _accel member variables in (rad/s) or (m/s^2), using ::_sensor_in
 */
bool APU_IMU_Oilpan::update () {
    int tc_temp = _adc->Ch(_gyro_temp_ch);

    // convert corrected gyro readings to angular velocities (rad/sec)
    //
    _gyro.x = ToRad(_gyro_gain_x) * _sensor_in(GYRO_X, tc_temp);
    _gyro.y = ToRad(_gyro_gain_y) * _sensor_in(GYRO_Y, tc_temp);
    _gyro.z = 0; // no sensor here!

    // convert corrected accelerometer readings to acceleration
    //
    _accel.x = _accel_scale * _sensor_in(ACCEL_X, tc_temp);
    _accel.y = _accel_scale * _sensor_in(ACCEL_Y, tc_temp);
    _accel.z = _accel_scale * _sensor_in(ACCEL_Z, tc_temp);

    // always updated
    return true;
}
