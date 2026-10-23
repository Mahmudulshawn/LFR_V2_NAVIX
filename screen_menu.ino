// ---------- DISPLAY ----------
void drawTitle(const char* t) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(t);
  display.drawFastHLine(0, 10, SCREEN_W, SSD1306_WHITE);
}

void drawMenu() {
  drawTitle("== MAIN MENU ==");
  for (int i = 0; i < MENU_COUNT; i++) {
    int y = 13 + i * 8;
    if (i == menuIndex) {
      display.fillRect(0, y - 1, SCREEN_W, 9, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(6, y);
    display.print(menuItems[i]);
  }
  display.display();
}

void drawStartStop(const char* title) {          // Run & Calibrate screens
  drawTitle(title);
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print(active ? "RUNNING" : "STOPPED");
  display.setTextSize(1);
  display.setCursor(0, 44);
  display.print(active ? "Click: stop" : "Click: start");
  display.setCursor(0, 54);
  display.print("Hold: menu");
  display.display();
}

void drawCalibScreen() {                 // idle screen
  drawTitle("CALIBRATE");
  display.setCursor(0, 22); display.print("Click: start");
  display.setCursor(0, 54); display.print("Hold: menu");
  display.display();
}

// CALIBRATION
void runCalibration() {
  active = true;
  drawTitle("CALIBRATE");
  display.setCursor(0, 28); display.print("Calibrating...");
  display.display();

  bool ok = calibration();               // returns true if every sensor saw enough contrast

  active = false;
  drawTitle("CALIBRATE");
  if (ok) {
    display.setCursor(0, 24); display.print("Calibration");
    display.setCursor(0, 38); display.print("successful!");
  } else {
    display.setCursor(0, 24); display.print("Calibration failed");
    display.setCursor(0, 38); display.print("Old values kept");
  }
  display.display();
  delay(1000);
  goToMenu();
}

// PRIORITY
void drawPriority() {
  drawTitle("TURN PRIORITY");
  const char* names[2] = {"Left", "Right"};
  for (int i = 0; i < 2; i++) {
    int y = 16 + i * 14;
    if (i == priorityCursor) {
      display.fillRect(0, y - 2, SCREEN_W, 12, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }
    display.setCursor(6, y);
    display.print(names[i]);
    if (i == priority) display.print("  *");   // * marks the currently saved choice
  }
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 46); display.print("1x: switch  2x: save");
  display.setCursor(0, 56); display.print("Hold: back");
  display.display();
}

void selectPriority(ButtonEvent ev) {
  if (ev == EV_SINGLE) {
    priorityCursor = (priorityCursor == PRIORITY_LEFT) ? PRIORITY_RIGHT : PRIORITY_LEFT;
    drawPriority();
  }
  else if (ev == EV_DOUBLE) {
    priority = priorityCursor;                 // save
    drawTitle("TURN PRIORITY");
    display.setCursor(0, 28);
    display.print("Saved: ");
    display.print(priority == PRIORITY_LEFT ? "Left" : "Right");
    display.display();
    delay(600);
    goToMenu();
  }
}

// PID
void drawPIDScreen() {
  drawTitle(pidEditing ? "PID  [EDIT]" : "PID");
  const char* names[7] = {"P", "D", "Gap", "Adv", "PvtSp", "Speed"};

  for (int i = 0; i < 6; i++) {
    int y = 12 + i * 8;
    bool editingThis = (pidEditing && i == pidField);

    if (editingThis) {                                 // inverted row = being edited
      display.fillRect(0, y - 2, SCREEN_W, 11, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
    } else {
      display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(0, y);
    display.print((i == pidField && !pidEditing) ? ">" : " ");
    display.print(names[i]);
    display.print(": ");
    if (i == 0) display.print(p, 2);
    else if (i == 1) display.print(d, 2);
    else if (i == 2) display.print(GAP_DRIVE_MS);
    else if (i == 3) display.print(ADVANCE_MS);
    else if (i == 4) display.print(PIVOT_SPEED);
    else display.print(BASE_SPEED);
  }

  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void adjustPID(int dir) {                  // dir = +1 or -1
  if (pidField == 0)      p = constrain(p + dir * 0.5f, 0.00f, 100.00f);
  else if (pidField == 1) d = constrain(d + dir * 0.5f, 0.0f, 200.0f);
  else if (pidField == 2) GAP_DRIVE_MS = constrain(GAP_DRIVE_MS + dir * 5, 0, 500);
  else if (pidField == 3) ADVANCE_MS = constrain(ADVANCE_MS + dir * 10, 0, 500);
  else if (pidField == 4) PIVOT_SPEED = constrain(PIVOT_SPEED + dir * 5, 0, 500);
  else                    BASE_SPEED = constrain(BASE_SPEED + dir * 10, 0, MAX_SPEED);
}

void selectPID(ButtonEvent ev) {
  if (ev == EV_NONE) return;

  if (!pidEditing) {
    if (ev == EV_SINGLE)      pidField = (pidField + 1) % 6;
    else if (ev == EV_DOUBLE) pidEditing = true;
  } else {
    if (ev == EV_SINGLE)      adjustPID(+1);
    else if (ev == EV_TRIPLE) adjustPID(-1);
    else if (ev == EV_DOUBLE) pidEditing = false;
  }
  drawPIDScreen();
}

void drawSensorScreen() {
  drawTitle("SENSORS");
  printSensor();
  display.display();
}

void redraw() {
  switch (state) {
    case ST_MENU:     drawMenu(); break;
    case ST_RUN:      drawStartStop("RUN"); break;
    case ST_CALIB:    drawCalibScreen(); break;
    case ST_PRIORITY: drawPriority(); break;
    case ST_PID:      drawPIDScreen(); break;
    case ST_SENSOR:   drawSensorScreen(); break;
    case ST_DEBUG:    drawDebugScreen(); break; 
  }
}

// ---------- NAVIGATION ----------
void goToMenu() {
  stopMotors();
  active = false;
  pidEditing = false;
  multiClickEnabled = true;
  state = ST_MENU;
  redraw();
}

void enterSelected() {
  state = (State)(menuIndex + 1);     // order matches menuItems: Run, Calib, Priority, PID, Sensor
  active = false;

  if (state == ST_RUN) multiClickEnabled = false;   // instant start/stop response
  if (state == ST_SENSOR) active = true;            // sensor starts immediately
  if (state == ST_PRIORITY) priorityCursor = priority;
  if (state == ST_PID) { pidField = 0; pidEditing = false; }
  if (state == ST_DEBUG) logScroll = 0;

  redraw();
}
