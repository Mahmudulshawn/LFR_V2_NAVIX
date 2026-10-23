void leftMotor(int speed)
{
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

  if (speed > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  }
  else
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);
  }
}

void rightMotor(int speed)
{
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

  if (speed > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -speed);
  }
  else
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
  }
}

void setMotors(int l, int r) { leftMotor(l); rightMotor(r); }

// ============
// TURN LOGICS
// ============
void straight(int speed)
{
  speed = constrain(speed, 0, MAX_SPEED);

  leftMotor(speed);
  rightMotor(speed);
}

// Move backward
void backward(int speed)
{
  speed = constrain(speed, 0, MAX_SPEED);

  leftMotor(-speed);
  rightMotor(-speed);
}

// Turn left
void left(int speed)
{
  speed = constrain(speed, 0, MAX_SPEED);

  leftMotor(-speed);
  rightMotor(speed);
}

// Turn right
void right(int speed) 
{
  speed = constrain(speed, 0, MAX_SPEED);

  leftMotor(speed);
  rightMotor(-speed);
}

// Stop
void stopMotors()
{
  leftMotor(0);
  rightMotor(0);
}
