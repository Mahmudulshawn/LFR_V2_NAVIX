#include <stdarg.h>

#define LOG_LINES 24
#define LOG_LEN   22          // 21 characters fit on the screen
#define LOG_ROWS  6           // lines visible at once

char logBuf[LOG_LINES][LOG_LEN];
int  logHead = 0;             // next slot to write
int  logCount = 0;
int  logScroll = 0;           // 0 = newest page

void dbg(const char* fmt, ...) {              // like printf, but into the buffer
  va_list args;
  va_start(args, fmt);
  vsnprintf(logBuf[logHead], LOG_LEN, fmt, args);
  va_end(args);
  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES) logCount++;
}

void dbgClear() { logHead = 0; logCount = 0; logScroll = 0; }

void dbgBits(uint16_t b) {                    // sensor 0 first, same order as the Sensor screen
  char s[17];
  for (int i = 0; i < 16; i++) s[i] = ((b >> i) & 1) ? '1' : '0';
  s[16] = 0;
  dbg("%s", s);
}

void drawDebugScreen() {
  drawTitle("LOG 1x:older 2x:clr");
  if (logCount == 0) { display.setCursor(0, 24); display.print("(empty)"); }
  for (int r = 0; r < LOG_ROWS; r++) {
    int n = logScroll + r;                    // 0 = newest
    if (n >= logCount) break;
    int idx = (logHead - 1 - n + LOG_LINES * 2) % LOG_LINES;
    display.setCursor(0, 13 + r * 8);
    display.print(logBuf[idx]);
  }
  display.display();
}

void selectDebug(ButtonEvent ev) {
  if (ev == EV_SINGLE) {                      // page back through older entries, wraps to newest
    logScroll += LOG_ROWS;
    if (logScroll >= logCount) logScroll = 0;
    drawDebugScreen();
  } else if (ev == EV_DOUBLE) {
    dbgClear();
    drawDebugScreen();
  }
}