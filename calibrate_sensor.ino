bool calibration() {
  for (int i = 0; i < SENSOR_NUMBER; i++) { Min_Value[i] = 4095; Max_Value[i] = 0; }

  const int dir[4] = {-1, 1, 1, -1};
  for (int step = 0; step < 4; step++) {
    stopMotors();
    delay(200);
    setMotors(CAL_SPEED * dir[step], -CAL_SPEED * dir[step]);
    uint32_t t0 = millis();
    while (millis() - t0 < CAL_STEP_MS) {
      read_black_line();
      for (int i = 0; i < SENSOR_NUMBER; i++) {
        Min_Value[i] = min(Min_Value[i], sensorADC[i]);
        Max_Value[i] = max(Max_Value[i], sensorADC[i]);
      }
    }
  }
  stopMotors();

  for (int i = 0; i < SENSOR_NUMBER; i++){
    if (DISABLED_MASK & (1u << i)) continue;
    if (Max_Value[i] - Min_Value[i] < CAL_MIN_SPAN) return false;   // keep old thresholds
  }
  for (int i = 0; i < SENSOR_NUMBER; i++)
    sensorThreshold[i] = (Min_Value[i] + Max_Value[i]) / 2;
  
  saveCalibration();
  return true;
}

void saveCalibration() {
  eeprom_buffer_fill();
  eeprom_buffered_write_byte(EEPROM_ADDR, EEPROM_MAGIC);
  for (int i = 0; i < SENSOR_NUMBER; i++) {
    eeprom_buffered_write_byte(EEPROM_ADDR + 1 + i * 2, sensorThreshold[i] & 0xFF);
    eeprom_buffered_write_byte(EEPROM_ADDR + 2 + i * 2, (sensorThreshold[i] >> 8) & 0xFF);
  }
  eeprom_buffer_flush();                       // single flash write
}

bool loadCalibration() {
  eeprom_buffer_fill();
  if (eeprom_buffered_read_byte(EEPROM_ADDR) != EEPROM_MAGIC) return false;   // nothing saved yet
  for (int i = 0; i < SENSOR_NUMBER; i++) {
    sensorThreshold[i] = eeprom_buffered_read_byte(EEPROM_ADDR + 1 + i * 2)
                       | (eeprom_buffered_read_byte(EEPROM_ADDR + 2 + i * 2) << 8);
  }
  return true;
}