#include <Wire.h>
#include <Servo.h>
#include <SFE_BMP180.h>

SFE_BMP180 altimeter;
double baseline_pressure;
const int mpu_address = 0x68;

int16_t ax, ay, az, temperature, gx, gy, gz;

const double K = 0.4;
const double rate_dt = 5;
const double attitude_dt = 50;
const double altitude_dt = 100;

double measured_roll, measured_pitch, measured_altitude;
double measured_roll_rate, measured_pitch_rate, measured_yaw_rate;
double target_roll, target_pitch, target_altitude;
double target_roll_rate, target_pitch_rate, target_yaw_rate;
double corrected_roll_rate, corrected_pitch_rate, corrected_yaw_rate, corrected_force;

Servo motor[4];
double motor_target[4];

double attention;

void setup()
{
  Serial.begin(9600);
  init_sensors();
  init_motors();
  delay(5000);
}

void sample_controller()
{
  /* For use with a NeuroSky MindWave */
  update_attention();
  target_roll = 0;
  target_pitch = 0;
  target_yaw_rate = 0;
  target_altitude += (attention - 7.0) / 7.0;
}

void loop()
{
  update_sensors();
  mot0(motor_target[0]);
  mot1(motor_target[1]);
  mot2(motor_target[2]);
  mot3(motor_target[3]);
  delay(rate_dt);
}
