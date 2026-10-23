void read_black_line() 
{
  sumOnSensor = 0;
  sensorWeight = 0;
  bitSensor = 0;
  for(int i = 0; i < SENSOR_NUMBER; i++)
  {
    selectChannel(i);
    delayMicroseconds(20);
    sensorADC[i] = analogRead(SIG_PIN);
    
    sensorDigital[i] = (sensorADC[i] < sensorThreshold[i]) ? 1 : 0;
    if (DISABLED_MASK & (1u << i)) sensorDigital[i] = 0;

    // sumOnSensors
    sumOnSensor += sensorDigital[i];

    // weight
    sensorWeight += sensorDigital[i] * weightValue[i];

    // bit representation
    bitSensor += sensorDigital[i] * bitWeight[i];
  }
  if (sumOnSensor > 0) line_position = (float)sensorWeight / sumOnSensor;
}

void selectChannel(int channel)
{
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
}

void printSensor() 
{
  read_black_line();
  display.setTextSize(1);
  display.setCursor(15, 16);

  for(int i = 0; i < SENSOR_NUMBER; i++)
  {
    // Serial.print(String(sensorADC[i]) + "\t");
    display.print(sensorDigital[i]);
  }
  // Serial.println();
  display.println();

  display.setCursor(0, 30);
  display.println("sumOnSensor: " + String(sumOnSensor));
  display.setCursor(0, 40);
  display.println("sensorWeight: " + String(sensorWeight));
  display.setCursor(0, 50);
  display.println("bitSensor: " + String(bitSensor));
}