ButtonEvent readButton() {
  static bool lastRaw = HIGH, stable = HIGH, longFired = false;
  static uint32_t lastChange = 0, pressStart = 0, lastRelease = 0;
  static uint8_t clicks = 0;
  uint32_t now = millis();

  bool raw = digitalRead(BTN_PIN);
  if (raw != lastRaw) { lastRaw = raw; lastChange = now; }

  if (now - lastChange > DEBOUNCE_MS && raw != stable) {
    stable = raw;
    if (stable == LOW) {                         // pressed
      pressStart = now;
      longFired = false;
    } else if (!longFired) {                     // released (not after a long press)
      clicks++;
      lastRelease = now;
      if (!multiClickEnabled) { clicks = 0; return EV_SINGLE; }
    }
  }

  // long press fires while still held
  if (stable == LOW && !longFired && now - pressStart > LONG_PRESS_MS) {
    longFired = true;
    clicks = 0;
    return EV_LONG;
  }

  // click group finished
  if (stable == HIGH && clicks > 0 && now - lastRelease > MULTI_GAP_MS) {
    uint8_t c = clicks;
    clicks = 0;
    if (c == 1) return EV_SINGLE;
    if (c == 2) return EV_DOUBLE;
    return EV_TRIPLE;                            // 3 or more
  }
  return EV_NONE;
}
