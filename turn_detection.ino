uint8_t classifySample(uint16_t bits) {
  const uint16_t HALF_LOW = 0x00FF, HALF_HIGH = 0xFF00;                 // sensors 0-7 / 8-15
  uint16_t leftHalf  = (LEFT_MASK  & HALF_LOW) ? HALF_LOW : HALF_HIGH;  // half containing the left edge
  uint16_t rightHalf = (RIGHT_MASK & HALF_LOW) ? HALF_LOW : HALF_HIGH;

  uint8_t f = 0;
  if (__builtin_popcount(bits & LEFT_MASK)  >= EDGE_MIN) f |= JUNC_LEFT;
  if (__builtin_popcount(bits & RIGHT_MASK) >= EDGE_MIN) f |= JUNC_RIGHT;
  if (bits & CENTER_MASK)                                f |= JUNC_STRAIGHT;
  return f;
}

bool isFinishBox() { return __builtin_popcount((uint16_t)bitSensor) >= FINISH_MIN; }
bool isLineLost()  { return sumOnSensor == 0; }

// Creep forward, latch every side branch seen, then check if the line continues ahead.
uint8_t detectJunction() {
  uint8_t flags = 0;
  setMotors(TURN_SPEED, TURN_SPEED);
  uint32_t t0 = millis();
  while (millis() - t0 < ADVANCE_MS) {
    read_black_line();
    flags |= classifySample(bitSensor) & (JUNC_LEFT | JUNC_RIGHT);
  }
  read_black_line();
  if (isFinishBox()) flags |= JUNC_FINISH;
  flags |= classifySample(bitSensor) & JUNC_STRAIGHT;
  return flags;
}

// ----- MANUAL CASES: runs first. Return true if you handled it and set `act`. -----
bool customCases(uint8_t flags, Action &act) {

  // Example A: exact sensor pattern (use bitSensor)
  // if (bitSensor == 0b1111111010000000 | bitSensor == 0b0111111010000000) { act = ACT_LEFT; return true; }
  // if (bitSensor == 0b0000001011111111 | bitSensor == 0b0000001011111110) { act = ACT_RIGHT; return true; }

  // Example B: by junction number (junctionCount is already incremented)
  // if (junctionCount == 3) { act = ACT_PID; return true; }        // skip the 3rd junction

  // Example C: by flags
  // if (flags == (JUNC_LEFT | JUNC_RIGHT)) { act = ACT_LEFT; return true; }  // T-junction: always left

  // Example D: combine with priority
  // if (junctionCount == 5 && priority == PRIORITY_RIGHT) { act = ACT_UTURN; return true; }

  return false;   // not handled, use the default table below
}

Action decideAction(uint8_t flags) {
  if (flags & JUNC_FINISH) return ACT_STOP;
  if (!(flags & (JUNC_LEFT | JUNC_RIGHT))) return ACT_PID;
  Action act = ACT_PID;
  if (customCases(flags, act)) return act;

  Action pri = (priority == PRIORITY_LEFT) ? ACT_LEFT : ACT_RIGHT;

  switch (flags) {
    case 0:                                   // nothing seen (false trigger)
      return ACT_PID;

    case JUNC_STRAIGHT:                       // only straight (shouldn't trigger)
      return ACT_PID;

    case JUNC_LEFT:                           // 90° left corner, no way ahead
      return ACT_LEFT;

    case JUNC_RIGHT:                          // 90° right corner
      return ACT_RIGHT;

    case JUNC_LEFT | JUNC_STRAIGHT:           // branch left + straight
      return (priority == PRIORITY_LEFT) ? ACT_LEFT : ACT_PID;

    case JUNC_RIGHT | JUNC_STRAIGHT:          // branch right + straight
      return (priority == PRIORITY_RIGHT) ? ACT_RIGHT : ACT_PID;

    case JUNC_LEFT | JUNC_RIGHT:              // T-junction, no straight
      return pri;

    case JUNC_LEFT | JUNC_RIGHT | JUNC_STRAIGHT:   // crossroad
      return pri;
  }
  return ACT_PID;
}

// Pivot until the old line leaves the center, then until a line returns to the center.
// bool turn(PRIORITY dir, uint32_t timeout) {
//   int l = (dir == PRIORITY_LEFT) ? -TURN_SPEED : TURN_SPEED;
//   setMotors(l, -l);
  
//   uint32_t t0 = millis();
//   while (millis() - t0 < timeout) {            // phase 1: leave the old line
//     read_black_line();
//     if (!(bitSensor & CENTER_MASK)) break;
//     if (digitalRead(BTN_PIN) == LOW) return false;
//   }
//   while (millis() - t0 < timeout) {            // phase 2: find the new line
//     read_black_line();
//     if (bitSensor & CENTER_MASK) return true;  // Found it, success
//     if (digitalRead(BTN_PIN) == LOW) { stopMotors(); return false; }
//   }

//   stopMotors();                                // Timed out, no line found
//   return true;                                 // true - Keep looking for line (false if want to stop after turn)
// }

bool turn(PRIORITY dir, uint32_t timeout) {
  int l = (dir == PRIORITY_LEFT) ? -PIVOT_SPEED : PIVOT_SPEED;
  setMotors(l, -l);

  uint32_t t0 = millis();
  uint8_t phase = 1;                                // 1 = leave old line, 2 = find new line
  while (millis() - t0 < timeout) {
    read_black_line();
    bool centerOn = bitSensor & CENTER_MASK;

    if (phase == 1 && !centerOn) phase = 2;
    else if (phase == 2 && centerOn) {
      dbg("T%c ok %lums", dir == PRIORITY_LEFT ? 'L' : 'R', (unsigned long)(millis() - t0));
      return true;
    }
    if (digitalRead(BTN_PIN) == LOW) { stopMotors(); return false; }
  }

  stopMotors();
  dbg("T%c TIMEOUT p%d", dir == PRIORITY_LEFT ? 'L' : 'R', phase);
  return true;
}

void reverseToLine(uint32_t timeout) {
  setMotors(-TURN_SPEED, -TURN_SPEED);
  uint32_t t0 = millis();
  while (millis() - t0 < timeout) {
    read_black_line();
    if (sumOnSensor > 0) break;
    if (digitalRead(BTN_PIN) == LOW) break;
  }
  stopMotors();
}

void stopRun() {
  stopMotors();
  active = false;
  drawStartStop("RUN");
}

void handleIntersection() {
  uint16_t trig = (uint16_t)bitSensor;     // the sample that triggered this junction
  junctionCount++;
  uint8_t flags = detectJunction();
  Action act = decideAction(flags);

  dbgBits(trig);
  dbg("J%d f%d a%d", junctionCount, (int)flags, (int)act);

  bool ok = true;
  switch (act) {
    case ACT_LEFT:  ok = turn(PRIORITY_LEFT,  TURN_TIMEOUT_MS);  break;
    case ACT_RIGHT: ok = turn(PRIORITY_RIGHT, TURN_TIMEOUT_MS);  break;
    case ACT_UTURN: ok = turn(PRIORITY_LEFT,  UTURN_TIMEOUT_MS); break;
    case ACT_STOP:  abortRun(); return;
    case ACT_PID:   break;               // just keep following the line
  }
  if (!ok) { abortRun(); return; }

  previous_error = 0;
  lastJunction = millis();
}

void abortRun() {
  stopRun();
  while (digitalRead(BTN_PIN) == LOW) {}
  delay(DEBOUNCE_MS);
}

