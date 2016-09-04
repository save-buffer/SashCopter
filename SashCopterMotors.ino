int my_map(int throttle)
{
  int map1 = map(throttle, 0, 100, 80, 1023);
  return map(map1, 0, 1023, 0, 179);
}

void write_all(int throttle)
{
  throttle = my_map(throttle);
  for (int i = 0; i < 4; i++)
  {
    motor[i].write(throttle);
  }
}

void mot0(int throttle)
{
  motor[0].write(my_map(throttle));
}

void mot1(int throttle)
{
  motor[1].write(my_map(throttle));
}

void mot2(int throttle)
{
  motor[2].write(my_map(throttle));
}


void mot3(int throttle)
{
  motor[3].write(my_map(throttle));
}

void calc_motor_targets()
{
  motor_target[0] = K * (corrected_roll_rate + corrected_pitch_rate + corrected_yaw_rate + corrected_force);
  motor_target[1] = K * (-corrected_roll_rate + corrected_pitch_rate - corrected_yaw_rate + corrected_force);
  motor_target[2] = K * (-corrected_roll_rate - corrected_pitch_rate + corrected_yaw_rate + corrected_force);
  motor_target[3] = K * (corrected_roll_rate - corrected_pitch_rate - corrected_yaw_rate + corrected_force);
}

void init_motors()
{
  motor[0].attach(3);
  motor[1].attach(5);
  motor[2].attach(6);
  motor[3].attach(9);
  write_all(0);
}

