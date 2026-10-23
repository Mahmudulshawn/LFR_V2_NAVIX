void followLine() {
  read_black_line();

  if (isLineLost()) { handleLostLine(); return; }
  if (lostSince != 0) {
    dbg("G %lums", (unsigned long)(millis() - lostSince));
    previous_error = center_position - (float)sensorWeight / sumOnSensor;
    lostSince = 0;
  }

  bool cooled  = millis() - lastJunction > JUNCTION_COOLDOWN_MS;
  bool edgeHit = classifySample(bitSensor) & (JUNC_LEFT | JUNC_RIGHT);

  // 1) Edge sensors lit = junction/turn. Nothing else decides this.
  if (cooled && edgeHit) {
    freezeSince = 0;
    handleIntersection();            // creeps forward ADVANCE_MS, then decides
    return;
  }

  // 2) Bar much wider than a line but edges not lit yet: hold straight, don't let PID chase it.
  if (cooled && sumOnSensor > LINE_MAX_WIDTH) {
    if (freezeSince == 0) freezeSince = millis();
    if (millis() - freezeSince < FREEZE_MAX_MS) {
      setMotors(BASE_SPEED, BASE_SPEED);
      return;
    }
  } else {
    freezeSince = 0;
  }

  PID_Controller(BASE_SPEED, p, d);
}

void handleLostLine() {
  if (lostSince == 0) lostSince = millis();        // first all-white sample starts the clock
  uint32_t lostFor = millis() - lostSince;

  bool leftSide  = line_position < center_position - GAP_CENTER_ZONE;   // last seen near sensor 0 end
  bool rightSide = line_position > center_position + GAP_CENTER_ZONE;   // last seen near sensor 15 end
  bool ahead     = !leftSide && !rightSide;                              // last seen near the middle

  if (lostFor < LOST_CONFIRM_MS) {                 // not lost yet
    if (ahead) setMotors(BASE_SPEED, BASE_SPEED);  // line ended ahead: keep rolling
    return;                                        // off to a side: keep the last PID command
  }

  PRIORITY dir = PRIORITY_LEFT;                    // dead-end default
  if (ahead) {
    if (lostFor < GAP_DRIVE_MS) {                  // confirmed lost, but cross a possible gap first
      setMotors(BASE_SPEED, BASE_SPEED);
      reverseToLine(400);
      return;
    }
  } else {
    dir = leftSide ? PRIORITY_LEFT : PRIORITY_RIGHT;
  }

  dbg("L lp%d %s", (int)line_position, dir == PRIORITY_LEFT ? "left" : "right");

  lostSince = 0;
  if (turn(dir, UTURN_TIMEOUT_MS)) { previous_error = 0; lastJunction = millis(); }
  else abortRun();
}
