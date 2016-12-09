#define MOTOR_0 9
#define MOTOR_1 6
#define MOTOR_2 3
#define MOTOR_3 5

int my_map(int throttle)
{
  int map1 = map(throttle, 0, 100, 80, 1023);
  return map(map1, 0, 1023, 0, 179);
}

void write_all(int throttle)
{
  int mapped = my_map(throttle);
  motor0.write(mapped);
  motor1.write(mapped);
  motor2.write(mapped);
  motor3.write(mapped);
}

void mot0(int throttle)
{
  motor0.write(my_map(throttle));
}

void mot1(int throttle)
{
  motor1.write(my_map(throttle));
}

void mot2(int throttle)
{
  motor2.write(my_map(throttle));
}

void mot3(int throttle)
{
  motor3.write(my_map(throttle));
}

void calc_motor_targets()
{
  //Weight distribution is very off on my drone, so I add some extra motor strength in the back
  motor_target[0] = EQUILIBRIUM_SPEED + (corrected_roll_rate + corrected_pitch_rate + corrected_yaw_rate + corrected_force);
  motor_target[1] = EQUILIBRIUM_SPEED + (-corrected_roll_rate + corrected_pitch_rate - corrected_yaw_rate + corrected_force);
  motor_target[2] = EQUILIBRIUM_SPEED + (-corrected_roll_rate - corrected_pitch_rate + corrected_yaw_rate + corrected_force) + 1;
  motor_target[3] = EQUILIBRIUM_SPEED + (corrected_roll_rate - corrected_pitch_rate - corrected_yaw_rate + corrected_force) + 1;

  for (int i = 0; i < 4; i++)
  {
    if (motor_target[i] > MAX_MOTOR_SPEED)
    {
      motor_target[i] = MAX_MOTOR_SPEED;
    }
    else if (motor_target[i] < MIN_MOTOR_SPEED)
    {
      motor_target[i] = MIN_MOTOR_SPEED;
    }
  }
#if THROTTLE_SIGNALS == 1
  Serial.print("Motor0, Motor1, Motor2, Motor3: ");
  Serial.print((int)motor_target[0]);
  Serial.print("  ");
  Serial.print((int)motor_target[1]);
  Serial.print("  ");
  Serial.print((int)motor_target[2]);
  Serial.print("  ");
  Serial.println((int)motor_target[3]);
#endif
}

void init_motors()
{
  motor0.attach(MOTOR_0);
  motor1.attach(MOTOR_1);
  motor2.attach(MOTOR_2);
  motor3.attach(MOTOR_3);
  write_all(0);
}

void update_motors()
{
  calc_motor_targets();

  mot0(round(motor_target[0]));
  mot1(round(motor_target[1]));
  mot2(round(motor_target[2]));
  mot3(round(motor_target[3]));
}

