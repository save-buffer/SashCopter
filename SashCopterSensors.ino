

double get_pressure()
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
          return P;
      }
    }
  }
  return -1.0;
}

double get_temperature()
{
  char status = altimeter.startTemperature();
  if (status != 0)
  {
    delay(status);
    double T;
    status = altimeter.getTemperature(T);
    if (status != 0)
      return T;
  }
  return -1.0;
}

double get_pressure(double T)
{
  double P;
  char status = altimeter.startPressure(3);
  if (status != 0)
  {
    delay(status);
    status = altimeter.getPressure(P, T);

    if (status != 0)
      return P;
  }
  return -1.0;
}

double get_ultrasonic_altitude()
{
  static double prev_ultrasonic_altitude = 0;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH);
  Serial.print("Duration: ");
  Serial.println(duration);
  /* Sometimes the ultrasonic gives ridiculous values (on the order of 167k) despite max value only being 23529 */
  if(duration > 23500)
    return prev_ultrasonic_altitude;
  prev_ultrasonic_altitude = ((double)duration * 0.034 / 2.0) / 100.0;
  return prev_ultrasonic_altitude;
}

double get_altitude(double Pressure)
{
#if ALTIMETER == 1
  return altimeter.altitude(Pressure, baseline_pressure);
#else
  return get_ultrasonic_altitude();
#endif

}

double get_altitude()
{

#if ALTIMETER == 1
  return (altimeter.altitude(get_pressure(), baseline_pressure) + altimeter.altitude(get_pressure(), baseline_pressure)
          + altimeter.altitude(get_pressure(), baseline_pressure) + altimeter.altitude(get_pressure(), baseline_pressure)) / 4.0;
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
  const double alpha = 0.98;
  double fax = (Wire.read() << 8 | Wire.read()) / 16384.0;
  double fay = (Wire.read() << 8 | Wire.read()) / 16384.0;
  double faz = (Wire.read() << 8 | Wire.read()) / 16384.0;
  temperature = ((Wire.read() << 8 | Wire.read()) / 340.0) + 36.53;
  gx = (Wire.read() << 8 | Wire.read()) / 131.0 + 15.0;
  gy = (Wire.read() << 8 | Wire.read()) / 131.0 + -1.0;
  gz = (Wire.read() << 8 | Wire.read()) / 131.0 + 2.0;
  ax = fax * alpha + (ax * (1.0 - alpha));
  ay = fay * alpha + (ay * (1.0 - alpha));
  az = faz * alpha + (az * (1.0 - alpha));

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

const double tau = 0.6;
double complementary_filter(double og_angle, double gyro_val, double accel_angle)
{
  return tau * (og_angle + ((double)attitude_dt / 1000.0) * gyro_val) + (1.0 - tau) * accel_angle;
}

void update_altitude(double T)
{
#if ALTIMETER == 1
  const double alpha = 0.8;
  avg_pressure = ((avg_pressure * p_count) + (get_pressure(T) + get_pressure(T) + get_pressure(T) + get_pressure(T))) / ((double)p_count + 4.0);
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
  double roll = (atan2(-ay, az) * 180.0) / 3.14159;
  double pitch = (atan2(ax, sqrt((ay * ay) + (az * az))) * 180.0) / 3.14159;
  mpu_roll = complementary_filter(mpu_roll, gx, roll);
  mpu_pitch = complementary_filter(mpu_pitch, gy, pitch);

  /*
      IMPORTANT: THE MPU IS TURNED BY 90 DEGREES ON THE DRONE. AS A RESULT, THE ROLL AND THE PITCH NEED TO BE SWITCHED. THIS IS SPECIFIC TO MY DRONE!
      measured_roll has also experimentally observed to have an offset of -5.5 degrees. Similarly, measured_pitch has an offset of -1.1 degrees.
  */
  measured_roll = -mpu_pitch + 5;
  measured_pitch = mpu_roll + 1.1;

#if ACCEL_CALC == 1
  Serial.print("Calculated accelerometer roll: ");
  Serial.println(roll);
  Serial.print("Calculated accelerometer pitch: ");
  Serial.println(pitch);
#endif
}

int at_count = 0;
int al_count = 0;
void update_sensors()
{
  const int attitude_loops = (int)(attitude_dt / rate_dt);
  const int altitude_loops = (int)(altitude_dt / rate_dt);
  update_attitude();
  //temperature = (get_temperature() + temperature) / 2.0;
  temperature = get_temperature();
  {
    at_count = 0;
    sample_controller();
    correct_attitude();
  }
  if (al_count >= altitude_loops)
  {
    update_altitude(temperature);
    al_count = 0;
    correct_altitude();
  }
  correct_rates();
  at_count++;
  al_count++;
}

