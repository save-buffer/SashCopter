void correct_rates()
{
  float e_roll = target_roll_rate - measured_roll_rate;
  float e_pitch = target_pitch_rate - measured_pitch_rate;
  float e_yaw = target_yaw_rate - measured_yaw_rate;

  corrected_roll_rate = roll_rate_controller(e_roll);
  corrected_pitch_rate = pitch_rate_controller(e_pitch);
  corrected_yaw_rate = yaw_rate_controller(e_yaw);

#if RATE_PID == 1
  Serial.print("corrected_roll_rate, corrected_pitch_rate, corrected_yaw_rate:  ");
  Serial.print(corrected_roll_rate);
  Serial.print("  ");
  Serial.print(corrected_pitch_rate);
  Serial.print("  ");
  Serial.println(corrected_yaw_rate);
#endif
}

void correct_attitude()
{
  float e_roll = target_roll - measured_roll;
  float e_pitch = target_pitch - measured_pitch;
  target_roll_rate += roll_controller(e_roll);
  target_pitch_rate += pitch_controller(e_pitch);
#if ATTITUDE_PID == 1
  Serial.print("Target roll rate, Target pitch rate: ");
  Serial.print(target_roll_rate);
  Serial.print("  ");
  Serial.println(target_pitch_rate);
#endif
}

void correct_altitude()
{
  float e_alt = target_altitude - measured_altitude;
  corrected_force = altitude_controller(e_alt);
#if ALTITUDE_PID == 1
  Serial.print("Corrected force: ");
  Serial.println(corrected_force);
#endif
}

float altitude_controller(float error)
{
  const float kp = 4;
  const float ki = 0.5f;
  const float kd = 0.1f;

  static float running_sum = 0;
  static float prev_error = 0;
  running_sum += error;
  float correction = kp * error + ki * ((float)altitude_dt / 1000) * running_sum + kd * (error - prev_error) / ((float)altitude_dt / 1000);
  prev_error = error;
  return correction;
}


float roll_controller(float error)
{
  const float kp = 0.08f;
  const float ki = 0.01f;
  const float kd = 0.005f;

  static float running_sum = 0;
  static float prev_error = 0;
  running_sum += error;
  float P = kp * error;
  float I = ki * ((float)attitude_dt / 1000.0f) * running_sum;
  float D = kd * (error - prev_error) / ((float)attitude_dt / 1000.0f);
  float correction = P + I + D;
  prev_error = error;
  return correction;
}

float pitch_controller(float error)
{
  const float kp = 0.08f;
  const float ki = 0.01f;
  const float kd = 0.005f;

  static float running_sum = 0;
  static float prev_error = 0;
  running_sum += error;
  float correction = kp * error + ki * ((float)attitude_dt / 1000) * running_sum + kd * (error - prev_error) / ((float)attitude_dt / 1000);
  prev_error = error;
  return correction;
}

float roll_rate_controller(float error)
{
  const float kp = 0.08f;
  const float ki = 0.01f;
  const float kd = 0.005f;

  static float running_sum = 0;
  static float prev_error = 0;
  running_sum += error;
  float correction = kp * error + ki * ((float)rate_dt / 1000) * running_sum + kd * (error - prev_error) / ((float)rate_dt / 1000);
  prev_error = error;
  return correction;
}

float pitch_rate_controller(float error)
{

  const float kp = 0.08f;
  const float ki = 0.01f;
  const float kd = 0.005f;

  static float running_sum = 0;
  static float prev_error = 0;
  running_sum += error;
  float correction = kp * error + ki * ((float)rate_dt / 1000) * running_sum + kd * (error - prev_error) / ((float)rate_dt / 1000);
  prev_error = error;
  return correction;
}

float yaw_rate_controller(float error)
{
  const float kp = 0.3f;
  const float ki = 0.1f;
  const float kd = 0.01f;

  static float running_sum = 0;
  static float prev_error = 0;
  running_sum += error;
  float correction = kp * error + ki * ((float)rate_dt / 1000) * running_sum + kd * (error - prev_error) / ((float)rate_dt / 1000);
  prev_error = error;
  return correction;
}
