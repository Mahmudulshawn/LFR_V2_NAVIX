void PID_Controller(int base_speed, float p, float d) {
  read_black_line();

  if(sumOnSensor > 0) {
    line_position = (float)sensorWeight / sumOnSensor;
  }
  error = center_position - line_position;

  derivative = error - previous_error;

  // Motor Mixing
  int left_motor_speed = base_speed - (error * p + derivative * d);
  int right_motor_speed = base_speed + (error * p + derivative * d);

  previous_error = error;

  setMotors(left_motor_speed, right_motor_speed);
}

