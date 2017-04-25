float convert_raw_accel(int raw)
{
  return (raw * 2.0f) / 32678.0f;
}

float convert_raw_gyro(int raw)
{
  return (raw * 250.0f) / 32678.0f;
}

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
#if ULTRASONIC == 1
  Serial.print("Duration: ");
  Serial.println(duration);
#endif
  /* Sometimes the ultrasonic gives ridiculous values (on the order of 167k) despite max value only being 23529 */
  if (duration > 23500)
    return prev_ultrasonic_altitude;
  return prev_ultrasonic_altitude = ((float)duration * 0.00017); //duration * 340 m/s / 1,000,000 us/s / 2
}

float get_bmp_altitude()
{
  return (float)altimeter.altitude(get_pressure(), baseline_pressure);
}

float get_altitude()
{
  float alt = get_ultrasonic_altitude();
  if (alt > 2.0f)
    return get_bmp_altitude();
  return alt;
}

void update_mpu()
{
  //Read from external sensor
  Wire.beginTransmission(mpu_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_address, 14, true);
  const float alpha = 0.98f;
  float fax = (Wire.read() << 8 | Wire.read()) / 16384.0f;
  float fay = (Wire.read() << 8 | Wire.read()) / 16384.0f;
  float faz = (Wire.read() << 8 | Wire.read()) / 16384.0f;
  temperature = ((Wire.read() << 8 | Wire.read()) / 340.0f) - 35.0f; //This should (theoretically) be garbage, if we turned the temperature sensor off earlier
  gx = (Wire.read() << 8 | Wire.read()) / 131.0f + 15.0f;
  gy = (Wire.read() << 8 | Wire.read()) / 131.0f + -1.0f;
  gz = (Wire.read() << 8 | Wire.read()) / 131.0f + 2.0f;
  ax = fax * alpha + (ax * (1.0f - alpha));
  ay = fay * alpha + (ay * (1.0f - alpha));
  az = faz * alpha + (az * (1.0f - alpha));

  CurieIMU.readMotionSensor(curie_ax, curie_ay, curie_az, curie_gx, curie_gy, curie_gz);

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
  CurieIMU.begin();
  CurieIMU.autoCalibrateGyroOffset();
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  const long hz = 1000 / rate_dt;
  CurieIMU.setGyroRate(hz);
  CurieIMU.setAccelerometerRate(hz);
  CurieIMU.setAccelerometerRange(2);
  CurieIMU.setGyroRange(250);
  filter.begin(hz);

  Wire.begin();
  Wire.beginTransmission(mpu_address);
  Wire.write(0x6B);
#if MPU_THERMOMETER == 1
  Wire.write(0);
#else
  Wire.write(0 | (1 << 3)); //disables the temperature sensor
#endif
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
#if CURIE_MPU == 1
  float c_ax = convert_raw_accel(curie_ax);
  float c_ay = convert_raw_accel(curie_ay);
  float c_az = convert_raw_accel(curie_az);
  float c_gx = convert_raw_gyro(curie_gx);
  float c_gy = convert_raw_gyro(curie_gy);
  float c_gz = convert_raw_gyro(curie_gz);

  float curie_roll = filter.getRoll();
  float curie_pitch = filter.getPitch();
  float curie_yaw = filter.getYaw();

  /*
      IMPORTANT: THE MPU IS TURNED BY 90 DEGREES ON THE DRONE. AS A RESULT, THE ROLL AND THE PITCH NEED TO BE SWITCHED. THIS IS SPECIFIC TO MY DRONE!
      measured_roll has also experimentally observed to have an offset of -5.5 degrees. Similarly, measured_pitch has an offset of -1.1 degrees.
  */

  //THIS WILL HAVE TO BE CHANGED DEPENDING ON THE ORIENTATION OF THE ARDUINO101


  measured_roll = (-mpu_pitch + 5 + curie_roll) / 2;
  measured_pitch = (mpu_roll + 1.1f + curie_pitch) / 2;
#else
  measured_roll = (-mpu_pitch + 5);
  measured_pitch = (mpu_roll + 1.1f);
#endif

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
  if (millis() - rate_time >= rate_dt)
  {
    correct_rates();
    rate_time = millis();
  }
  if (millis() - att_time >= attitude_dt)
  {
    sample_controller();
    correct_attitude();
    att_time = millis();
  }
  if (millis() - alt_time >= altitude_dt)
  {
#if MPU_THERMOMETER == 1
    update_altitude((get_temperature() + temperature) / 2.0f);
#else
    update_altitude(get_temperature());
#endif
    alt_time = millis();
  }
}
