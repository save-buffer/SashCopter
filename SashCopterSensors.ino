

float get_pressure()
{
  char status;
  double T, P;

  status = altimeter.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = altimeter.getTemperature(T);
    if (status != 0)
    {
      status = altimeter.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = altimeter.getPressure(P, T);

        if (status != 0)
          return (float)P;
      }
    }
  }
  return -1.0f;
}

float get_temperature()
{
  char status = altimeter.startTemperature();
  if (status != 0)
  {
    delay(status);
    double T;
    status = altimeter.getTemperature(T);
    if (status != 0)
      return (float)T;
  }
  return -1.0f;
}

float get_pressure(float T)
{
  double P;
  char status = altimeter.startPressure(3);
  if (status != 0)
  {
    delay(status);
    status = altimeter.getPressure(P, (double&)T);

    if (status != 0)
      return (float)P;
  }
  return -1.0f;
}

float get_ultrasonic_altitude()
{
  static float prev_ultrasonic_altitude = 0;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH);
  Serial.print("Duration: ");
  Serial.println(duration);
  /* Sometimes the ultrasonic gives ridiculous values (on the order of 167k) despite max value only being 23529 */
  if (duration > 23500)
    return prev_ultrasonic_altitude;
  prev_ultrasonic_altitude = ((float)duration * 0.034f / 2.0f) / 100.0f;
  return prev_ultrasonic_altitude;
}

float get_altitude(float Pressure)
{
#if ALTIMETER == 1
  return altimeter.altitude(Pressure, baseline_pressure);
#else
  return get_ultrasonic_altitude();
#endif

}

float get_altitude()
{

#if ALTIMETER == 1
  return (altimeter.altitude(get_pressure(), baseline_pressure) + altimeter.altitude(get_pressure(), baseline_pressure)
          + altimeter.altitude(get_pressure(), baseline_pressure) + altimeter.altitude(get_pressure(), baseline_pressure)) / 4.0f;
#else
  return get_ultrasonic_altitude();
#endif
}

void update_mpu()
{
  Wire.beginTransmission(mpu_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_address, 14, true);
  const float alpha = 0.98;
  float fax = (Wire.read() << 8 | Wire.read()) / 16384.0f;
  float fay = (Wire.read() << 8 | Wire.read()) / 16384.0f;
  float faz = (Wire.read() << 8 | Wire.read()) / 16384.0f;
  temperature = ((Wire.read() << 8 | Wire.read()) / 340.0f) + 35;
  gx = (Wire.read() << 8 | Wire.read()) / 131.0f + 15.0f;
  gy = (Wire.read() << 8 | Wire.read()) / 131.0f + -1.0f;
  gz = (Wire.read() << 8 | Wire.read()) / 131.0f + 2.0f;
  ax = fax * alpha + (ax * (1.0f - alpha));
  ay = fay * alpha + (ay * (1.0f - alpha));
  az = faz * alpha + (az * (1.0f - alpha));

#if MPU_RAW == 1
  Serial.print("fax, fay, faz, ax, ay, az, gx, gy, gz:    ");
  Serial.print(fax);
  Serial.print("  ");
  Serial.print(fay);
  Serial.print("  ");
  Serial.print(faz);
  Serial.print("  ");
  Serial.print(ax);
  Serial.print("  ");
  Serial.print(ay);
  Serial.print("  ");
  Serial.print(az);
  Serial.print("  ");
  Serial.print(gx);
  Serial.print("  ");
  Serial.print(gy);
  Serial.print("  ");
  Serial.println(gz);
  Serial.print("MPU Temperature: ");
  Serial.println(temperature);
#endif
}

void init_sensors()
{
  Wire.begin();
  Wire.beginTransmission(mpu_address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  if (altimeter.begin())
    Serial.println("BMP180 Success");

  baseline_pressure = get_pressure();
}

const float tau = 0.6;
float complementary_filter(float og_angle, float gyro_val, float accel_angle)
{
  return tau * (og_angle + ((float)attitude_dt / 1000.0f) * gyro_val) + (1.0f - tau) * accel_angle;
}

void update_altitude(float T)
{
#if ALTIMETER == 1
  const float alpha = 0.8;
  avg_pressure = ((avg_pressure * p_count) + (get_pressure(T) + get_pressure(T) + get_pressure(T) + get_pressure(T))) / ((float)p_count + 4.0f);
  measured_altitude = alpha * get_altitude(avg_pressure) + (1 - alpha) * measured_altitude;
#else
  measured_altitude = get_altitude();
#endif
}

void update_altitude()
{
  measured_altitude = get_altitude();
}

void update_attitude()
{
  update_mpu();
  measured_yaw_rate = gz; //we only measure yaw rate
  float roll = (atan2(-ay, az) * 180.0f) / 3.14159;
  float pitch = (atan2(ax, sqrt((ay * ay) + (az * az))) * 180.0f) / 3.14159;
  mpu_roll = complementary_filter(mpu_roll, gx, roll);
  mpu_pitch = complementary_filter(mpu_pitch, gy, pitch);

  /*
      IMPORTANT: THE MPU IS TURNED BY 90 DEGREES ON THE DRONE. AS A RESULT, THE ROLL AND THE PITCH NEED TO BE SWITCHED. THIS IS SPECIFIC TO MY DRONE!
      measured_roll has also experimentally observed to have an offset of -5.5 degrees. Similarly, measured_pitch has an offset of -1.1 degrees.
  */
  measured_roll = -mpu_pitch + 5;
  measured_pitch = mpu_roll + 1.1f;

#if ACCEL_CALC == 1
  Serial.print("Calculated accelerometer roll: ");
  Serial.println(roll);
  Serial.print("Calculated accelerometer pitch: ");
  Serial.println(pitch);
#endif
}

void update_sensors()
{
  static long alt_time = 0;
  static long rate_time = 0;
  static long att_time = 0;
  if(millis() - rate_time >= rate_dt)
  {
    update_rates();
    rate_time = millis();
  }
  if(millis() - att_time >= attitude_dt)
  {
    sample_controller();
    correct_attitude();
    att_time = millis();
  }
  if(millis() - alt_time >= altitude_dt)
  {
    //temperature = (get_temperature() + temperature) / 2.0f;
    temperature = get_temperature();
    update_altitude(temperature);
    alt_time = millis();
  }
}

