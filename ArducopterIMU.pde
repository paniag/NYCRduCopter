/*
 * Simple test for AP_IMU driver.
 *
 * Modified from AP_IMU example source for ArduCopterMega to work
 * with an Arduino Uno (ATMega328)
 *
 */
#include <AP_Math.h>
#include "board.h"
#include "APU_ADC.h"
#include "APU_IMU.h"

APU_ADC_Uno     adc;
APU_IMU_Oilpan  imu(&adc);

void setup ()
{
  pinMode(A_LED_PIN, OUTPUT);
  pinMode(C_LED_PIN, OUTPUT);
//  Serial.begin(38400);
  Serial.begin(9600);
  Serial.println("Doing IMU startup...");
  adc.Init();
  imu.init(IMU::COLD_START);
  Serial.println("Initialization complete");
}

void loop ()
{
  Vector3f  accel;
  Vector3f  gyro;

  delay(1000);
  imu.update();
  accel = imu.get_accel();
  gyro = imu.get_gyro();

  Serial.print("AX: ");
  Serial.print(accel.x);
  Serial.print("  AY: ");
  Serial.print(accel.y);
  Serial.print("  AZ: ");
  Serial.print(accel.z);
  Serial.print("  GX: ");
  Serial.print(gyro.x);
  Serial.print("  GY: ");
  Serial.println(gyro.y);
}
