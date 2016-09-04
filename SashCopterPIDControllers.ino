void correct_rates()
{
  double e_roll = target_roll_rate - measured_roll_rate;
  double e_pitch = target_pitch_rate - measured_pitch_rate;
  double e_yaw = target_yaw_rate - measured_yaw_rate;

  corrected_roll_rate = target_roll_rate + roll_rate_controller(e_roll);
  corrected_pitch_rate = pitch_rate_controller(e_pitch);
  corrected_yaw_rate = yaw_rate_controller(e_yaw);
}

void correct_attitude()
{
  double e_roll = target_roll - measured_roll;
  double e_pitch = target_pitch - measured_pitch;
  target_roll_rate += roll_controller(e_roll);
  target_pitch_rate += pitch_controller(e_pitch);
}

void correct_altitude()
{
  double e_alt = target_altitude - measured_altitude;
  corrected_force = altitude_controller(e_alt);
}

double altitude_controller(double error)
{
  const double kp = 1;
  const double ki = 0.5;
  const double kd = 0.1;
  
  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double correction = kp * error + ki * altitude_dt * running_sum + kd * (error - prev_error) / (altitude_dt / 1000);
  prev_error = error;
  return correction;
}


double roll_controller(double error)
{
  const double kp = 1;
  const double ki = 0.5;
  const double kd = 0.1;
  
  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double correction = kp * error + ki * attitude_dt * running_sum + kd * (error - prev_error) / (attitude_dt / 1000);
  prev_error = error;
  return correction;
}

double pitch_controller(double error)
{
  const double kp = 1;
  const double ki = 0.5;
  const double kd = 0.1;
  
  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double correction = kp * error + ki * attitude_dt * running_sum + kd * (error - prev_error) / (attitude_dt / 1000);
  prev_error = error;
  return correction;
}

double roll_rate_controller(double error)
{
  const double kp = 1;
  const double ki = 0.5;
  const double kd = 0.1;
  
  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double correction = kp * error + ki * rate_dt * running_sum + kd * (error - prev_error) / (rate_dt / 1000);
  prev_error = error;
  return correction;
}

double pitch_rate_controller(double error)
{
  const double kp = 1;
  const double ki = 0.5;
  const double kd = 0.1;
  
  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double correction = kp * error + ki * rate_dt * running_sum + kd * (error - prev_error) / (rate_dt / 1000);
  prev_error = error;
  return correction;
}

double yaw_rate_controller(double error)
{
  const double kp = 1;
  const double ki = 0.5;
  const double kd = 0.1;
  
  static double running_sum = 0;
  static double prev_error = 0;
  running_sum += error;
  double correction = kp * error + ki * rate_dt * running_sum + kd * (error - prev_error) / (rate_dt / 1000);
  prev_error = error;
  return correction;
}
