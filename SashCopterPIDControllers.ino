void correct_rates()
{
  double e_roll = target_roll_rate - measured_roll_rate;
  double e_pitch = target_pitch_rate - measured_pitch_rate;
  double e_yaw = target_yaw_rate - measured_yaw_rate;

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
  double e_roll = target_roll - measured_roll;
  double e_pitch = target_pitch - measured_pitch;
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
  double e_alt = target_altitude - measured_altitude;
  corrected_force = altitude_controller(e_alt);
#if ALTITUDE_PID == 1
  Serial.print("Corrected force: ");
  Serial.println(corrected_force);
#endif
}

double altitude_controller(double error)
{
  const double kp = 4;
  const double ki = 0.5;
  const double kd = 0.1;

  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double correction = kp * error + ki * ((double)altitude_dt / 1000) * running_sum + kd * (error - prev_error) / ((double)altitude_dt / 1000);
  prev_error = error;
  return correction;
}


double roll_controller(double error)
{
  const double kp = 0.08;
  const double ki = 0.01;
  const double kd = 0.005;

  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double P = kp * error;
  double I = ki * ((double)attitude_dt / 1000.0) * running_sum;
  double D = kd * (error - prev_error) / ((double)attitude_dt / 1000.0);
  double correction = P + I + D;
  prev_error = error;
  return correction;
}

double pitch_controller(double error)
{
  const double kp = 0.08;
  const double ki = 0.01;
  const double kd = 0.005;

  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double correction = kp * error + ki * ((double)attitude_dt / 1000) * running_sum + kd * (error - prev_error) / ((double)attitude_dt / 1000);
  prev_error = error;
  return correction;
}

double roll_rate_controller(double error)
{
  const double kp = 0.08;
  const double ki = 0.01;
  const double kd = 0.005;

  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double correction = kp * error + ki * ((double)rate_dt / 1000) * running_sum + kd * (error - prev_error) / ((double)rate_dt / 1000);
  prev_error = error;
  return correction;
}

double pitch_rate_controller(double error)
{

  const double kp = 0.08;
  const double ki = 0.01;
  const double kd = 0.005;

  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double correction = kp * error + ki * ((double)rate_dt / 1000) * running_sum + kd * (error - prev_error) / ((double)rate_dt / 1000);
  prev_error = error;
  return correction;
}

double yaw_rate_controller(double error)
{
  const double kp = 0.3;
  const double ki = 0.1;
  const double kd = 0.01;

  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double correction = kp * error + ki * ((double)rate_dt / 1000) * running_sum + kd * (error - prev_error) / ((double)rate_dt / 1000);
  prev_error = error;
  return correction;
}
