#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define EEPROM_ADDR 0
#define EEPROM_MAGIC 0xA5    // marks that valid calibration data is stored

#define SCREEN_W 128
#define SCREEN_H 64
#define OLED_SDA PB7
#define OLED_SCL PB6
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// SENSOR PINS
#define SENSOR_NUMBER 16
#define THRESHOLD 2200

#define SIG_PIN PA0
#define S0 PB12
#define S1 PB13
#define S2 PB14
#define S3 PB15

#define CAL_SPEED    110
#define CAL_STEP_MS  600
#define CAL_MIN_SPAN 400     // a sensor with less contrast than this = failed

int sensorADC[SENSOR_NUMBER];
int sensorDigital[SENSOR_NUMBER];
int weightValue[SENSOR_NUMBER] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160};
int bitWeight[SENSOR_NUMBER] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
int sensorThreshold[SENSOR_NUMBER];
int sumOnSensor;
int sensorWeight;
int bitSensor;

int Min_Value[SENSOR_NUMBER];
int Max_Value[SENSOR_NUMBER];

float line_position;
float error;
float previous_error;
float derivative;
float center_position = 85;

float p = 3.0;
float d = 20.0;

int pidField = 0;
bool pidEditing = false;

// LEFT MOTOR
#define PWMA PA6
#define AIN1 PB0
#define AIN2 PB1

// RIGHT MOTOR
#define PWMB PA7
#define BIN1 PB10
#define BIN2 PB2

#define MAX_SPEED 255
int BASE_SPEED = 110;

// #define UP PA1
// #define DOWN PA2
// #define SELECT PA3
// #define BACK PA4
// #define START PA8

#define BTN_PIN PA3
#define DEBOUNCE_MS   30
#define MULTI_GAP_MS  180           // max gap between clicks of a double/triple click
#define LONG_PRESS_MS 650

enum ButtonEvent { EV_NONE, EV_SINGLE, EV_DOUBLE, EV_TRIPLE, EV_LONG };
bool multiClickEnabled = true;      // false = single click fires instantly on release

// ---------- STATE ----------
enum State { ST_MENU, ST_RUN, ST_CALIB, ST_PRIORITY, ST_PID, ST_SENSOR, ST_DEBUG };
State state = ST_MENU;
bool active = false;                // true = Run / Calibrate / Sensor function is executing

const char* menuItems[] = {"Run", "Calibrate", "Priority", "PID", "Sensor", "Debug"};
const int MENU_COUNT = 6;
int menuIndex = 0;

enum PRIORITY { PRIORITY_LEFT, PRIORITY_RIGHT };
PRIORITY priority = PRIORITY_LEFT;        
PRIORITY priorityCursor = PRIORITY_LEFT;  // what's highlighted while in the submenu

// ---------- INTERSECTION ----------
#define SBIT(i)      (1u << (i))
#define LEFT_MASK    (SBIT(0)  | SBIT(1)  | SBIT(2))      // left end of the bar
#define RIGHT_MASK   (SBIT(13) | SBIT(14) | SBIT(15))     // right end of the bar

#define CENTER_MASK  0b0000001111000000   // sensors 6-9
#define DISABLED_MASK (1u << 7)   // Disable sensor 8 (7th index from left)
#define EDGE_MIN     2                    // Min active sensor to trigger turn
#define FINISH_MIN   14                   // sensors on = start/finish box
int ADVANCE_MS =     60;

#define TURN_SPEED   110
#define PIVOT_SPEED 130          // start here; TURN_SPEED now only sets the creep speed

#define TURN_TIMEOUT_MS  1500
#define UTURN_TIMEOUT_MS 2500
#define JUNCTION_COOLDOWN_MS 250
#define JUNC_MIN_ON 7          // Minimum active sensor needed to be a junction

#define LOST_GAP_MS 40         // gaps shorter than this are driven straight through
#define LOST_CONFIRM_MS 60    //(140) sumOnSensor must stay 0 this long before the line counts as lost
uint32_t lostSince = 0;

// Change GAP_DRIVE_MS depending on speed
int GAP_DRIVE_MS    =  60;   // how long to drive straight over a gap before giving up and U-turning
#define GAP_CENTER_ZONE 30    // last position within +/-30 of center = "ended straight ahead"

#define BAR_MIN 5             // sensors lit on that half of the bar for a real branch
#define LINE_MAX_WIDTH 4      // most sensors a normal line (even a 45° bend) lights
#define FREEZE_MAX_MS  120    // (120) max time to hold straight waiting for the edge sensors
uint32_t freezeSince = 0;

#define JUNC_LEFT      0x01
#define JUNC_RIGHT     0x02
#define JUNC_STRAIGHT  0x04
#define JUNC_FINISH 0x08

enum Action { ACT_PID, ACT_LEFT, ACT_RIGHT, ACT_UTURN, ACT_STOP };
uint32_t lastJunction = 0;
int junctionCount = 0;       // reset when Run starts, handy for manual cases

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(115200);
  analogReadResolution(12);
  loadCalibration();
  pinMode(BTN_PIN, INPUT_PULLUP);

  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  Wire.begin();
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println("OLED not found");
  }

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG_PIN, INPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  redraw();
}

void loop() {
  // put your main code here, to run repeatedly:
  ButtonEvent ev = readButton();

  // Long press = back to menu from anywhere
  if (ev == EV_LONG && state != ST_MENU) {
    goToMenu();
    ev = EV_NONE;
  }

  switch (state) {
    case ST_MENU:
      if (ev == EV_SINGLE) { menuIndex = (menuIndex + 1) % MENU_COUNT; redraw();}
      if (ev == EV_DOUBLE) { enterSelected(); } 
      break;

    case ST_RUN:
      if (ev == EV_SINGLE) {
        active = !active;
        if (active) { previous_error = 0; junctionCount = 0; lastJunction = millis(); lostSince = 0; freezeSince = 0; dbgClear(); }
        else stopMotors();
        drawStartStop("RUN");
      }
      if (active) followLine();
      break;

    case ST_CALIB:
      if (ev == EV_SINGLE) runCalibration();
      break;

    case ST_PRIORITY:
      selectPriority(ev);
      break;

    case ST_PID:
      selectPID(ev);
      break;

    case ST_SENSOR:
      if(active) {
        drawSensorScreen();
      }
      break;
    case ST_DEBUG:
      selectDebug(ev);
      break;
  }
}

