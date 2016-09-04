
double get_pressure()
{
  char status;
  double T, P, p0, a;

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
}

double get_altitude()
{
  double sum;
  for (int i = 0; i < 20; i++)
    sum += altimeter.altitude(get_pressure(), baseline_pressure);
  return (sum / 20);
}

void update_mpu()
{
  Wire.beginTransmission(mpu_address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_address, 14, true);
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();
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

const float tau = 0.075;
double complementary_filter(double gyro_angle, double accel_angle)
{
  return tau * gyro_angle + (1 - tau) * accel_angle;
}

void update_attitude()
{
  update_mpu();
  measured_roll += gx * attitude_dt;
  measured_pitch += gy * attitude_dt;
  measured_yaw_rate = gz; //we only measure yaw rate
  double roll = atan2(ay, az) * 180 / 3.14159;
  double pitch = atan(ax / sqrt((ay * ay) + (az * az))) * 180 / 3.14159;
  measured_roll = complementary_filter(measured_roll, roll);
  measured_pitch = complementary_filter(measured_pitch, pitch);
}

int at_count = 0;
int al_count = 0;
void update_sensors()
{
  const double attitude_loops = attitude_dt / rate_dt;
  const double altitude_loops = attitude_dt / rate_dt;
  update_attitude();
  if (at_count >= attitude_loops)
  {
    at_count = 0;
    sample_controller();
    correct_attitude();
  }
  if (al_count >= altitude_loops)
  {
    al_count = 0;
    correct_altitude();
  }
  correct_rates();
  at_count++;
  al_count++;
}

