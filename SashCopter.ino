#include <MadgwickAHRS.h>

#include <BMI160.h>
#include <CurieIMU.h>

#include <BLEAttribute.h>
#include <BLECentral.h>
#include <BLECharacteristic.h>
#include <BLECommon.h>
#include <BLEDescriptor.h>
#include <BLEPeripheral.h>
#include <BLEService.h>
#include <BLETypedCharacteristic.h>
#include <BLETypedCharacteristics.h>
#include <BLEUuid.h>
#include <CurieBLE.h>

#include <Wire.h>
#include <Servo.h>
#include <SPI.h>
#include <TFT.h>
#include <SFE_BMP180.h>

/* TFT Screen */
#define sd_cs  4
#define lcd_cs 10
#define dc     7
#define rst    8

/* Ultrasonic */
#define echo 14
#define trig 15

/* Debug Info */
#define ATTITUDE_PID 0
#define ALTITUDE_PID 0
#define RATE_PID 0
#define MPU_RAW 0
#define ACCEL_CALC 0
#define MEASURED_ANGLES 0
#define THROTTLE_SIGNALS 0
#define ULTRASONIC 0

/* Motors */
#define MAX_MOTOR_SPEED 60
#define MIN_MOTOR_SPEED 0
#define EQUILIBRIUM_SPEED 45
#define BACK_BONUS 5

/* In Seconds */
#define FLIGHT_TIME 10
/* If 1, will use BMP180. If 0, will use ultrasonic range finder */
#define ALTIMETER 0
#define MPU_THERMOMETER 1

TFT screen = TFT(lcd_cs, dc, rst);

BLEPeripheral board;
BLEService data("100D");
BLECharacteristic("2A38, BLERead | BLENotify, 

SFE_BMP180 altimeter;
float baseline_pressure;
const int mpu_address = 0x68;

Madgwick filter;

float ax, ay, az, temperature, gx, gy, gz;
int curie_ax, curie_ay, curie_az, curie_temperature, curie_gx, curie_gy, curie_gz;

const float K = 1.0f;
const long rate_dt = 30;
const long attitude_dt = 60;
const long altitude_dt = 90;

int p_count;
float avg_pressure;
float mpu_roll, mpu_pitch;
float measured_roll, measured_pitch, measured_altitude;
float measured_roll_rate, measured_pitch_rate, measured_yaw_rate;
float target_roll, target_pitch, target_altitude;
float target_roll_rate, target_pitch_rate, target_yaw_rate;
float corrected_roll_rate, corrected_pitch_rate, corrected_yaw_rate, corrected_force;

Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;
float motor_target[4];

float attention;

void sample_controller()
{
  /* For use with a NeuroSky MindWave */
  //update_attention();
  target_roll = 0;
  target_pitch = 0;
  target_yaw_rate = 0;
  target_altitude = 0.3;
  //target_altitude += (attention - 7.0f) / 7.0f;
}

void setup()
{
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  screen.begin();
  screen.background(0, 0, 0);
  screen.stroke(255, 255, 255);
  screen.setTextSize(1);
  Serial.begin(9600);
  init_sensors();
  init_motors();
  p_count = 0;
  do
  {
    p_count++;
    avg_pressure += get_pressure();
  } while (millis() < 6000);
  avg_pressure /= p_count;
  baseline_pressure = avg_pressure;
  Serial.print("Baseline Pressure: ");
  Serial.println(baseline_pressure);
  Serial.print("Baseline altitude: ");
  Serial.println(get_altitude());
}


void loop()
{
  static bool landing = false;
  update_sensors();
  update_motors();

  if (millis() / 1000 >= 15 + FLIGHT_TIME)
  {
    write_all(0);
    screen.background(0, 0, 0);
    String x = "Forced landing.\nAltitude: ";
    x += measured_altitude;
    screen.text(x.c_str(), 0, 0);
    for (;;);
  }
  if (millis() / 1000 >= 5 + FLIGHT_TIME)
  {
    target_altitude = 0.0f;
    Serial.print("Beginning landing. Altitude: ");
    Serial.println(measured_altitude);
    landing = true;
  }
  if (landing)
  {
    if (measured_altitude <= 0.1)
    {
      write_all(0);
      screen.background(0, 0, 0);
      screen.text("Landed!", 0, 0);
      Serial.println("Landed");
      for (;;);
    }
  }
  static String data("");
  screen.stroke(0, 0, 0);
  screen.text(data.c_str(), 0, 0);
  screen.stroke(255, 255, 255);
  data = "Measured Roll: ";
  if (landing)
    data = "Landing.\n" + data;
  data += measured_roll;
  data += "\nMeasured Pitch: ";
  data += measured_pitch;
  data += "\nMeasured Altitude: ";
  data += measured_altitude;
  data += "\nMotor 0: ";
  data += motor_target[0];
  data += "\nMotor 1: ";
  data += motor_target[1];
  data += "\nMotor 2: ";
  data += motor_target[2];
  data += "\nMotor 3: ";
  data += motor_target[3];
  screen.text(data.c_str(), 0, 0);
#if MEASURED_ANGLES == 1
  Serial.print("Measured Roll, Pitch, Altitude:  ");
  Serial.print(measured_roll
  Serial.print("    ");
  Serial.print(measured_pitch);
  Serial.print("    ");
  Serial.println(measured_altitude);
#endif
}
