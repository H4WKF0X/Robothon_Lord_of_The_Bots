#include <MeAuriga.h>

// ── Ports ─────────────────────────────────────────────────────────────────────
#define COLOR_PORT         PORT_7
#define LINE_FOLLOWER_PORT PORT_6
#define MOTOR_LEFT_SLOT    SLOT_1
#define MOTOR_RIGHT_SLOT   SLOT_2

// ── Tuning ────────────────────────────────────────────────────────────────────
const int BASE_SPEED = 70;
const int TURN_SOFT  = 50;
const int TURN_HARD  = 40;
const int LOOP_DELAY = 10;

const int RED_R_MIN = 80;
const int RED_G_MAX = 60;
const int RED_B_MAX = 60;

// ── Objects ───────────────────────────────────────────────────────────────────
MeLineFollower   lineSensor(LINE_FOLLOWER_PORT);
MeColorSensor    colorSensor(COLOR_PORT);
MeEncoderOnBoard motorLeft(MOTOR_LEFT_SLOT);
MeEncoderOnBoard motorRight(MOTOR_RIGHT_SLOT);

// ── Motors ────────────────────────────────────────────────────────────────────
static void _setMotors(int l, int r) {
  motorLeft.setMotorPwm(-l);
  motorRight.setMotorPwm(r);
}
static void _stop() { _setMotors(0, 0); }

// ── Color sensor ──────────────────────────────────────────────────────────────
bool colorIsRed() {
  colorSensor.ColorDataRead();
  uint16_t r = colorSensor.ReturnRedData();
  uint16_t g = colorSensor.ReturnGreenData();
  uint16_t b = colorSensor.ReturnBlueData();
  return (r > RED_R_MIN && g < RED_G_MAX && b < RED_B_MAX);
}

// ── Line follower ─────────────────────────────────────────────────────────────
void followLine() {
  switch (lineSensor.readSensors()) {
    case S1_IN_S2_IN:   _setMotors(BASE_SPEED, BASE_SPEED - TURN_SOFT); break;
    case S1_IN_S2_OUT:  _setMotors(BASE_SPEED, BASE_SPEED);              break;
    case S1_OUT_S2_IN:  _setMotors(BASE_SPEED - TURN_HARD, BASE_SPEED); break;
    case S1_OUT_S2_OUT: _setMotors(BASE_SPEED - TURN_SOFT, BASE_SPEED); break;
    default:            _stop();                                          break;
  }
}

// ── Setup & loop ──────────────────────────────────────────────────────────────
static bool stopped    = false;
static int  redConfirm = 0;

void setup() {
  colorSensor.SensorInit();
  _stop();
  delay(2000);
}

void loop() {
  if (stopped) return;

  if (colorIsRed()) {
    _stop();  // halt immediately on first red detect
    redConfirm++;
    if (redConfirm >= 3) {
      stopped = true;  // confirmed red → stay stopped
    }
  } else {
    redConfirm = 0;
    followLine();
  }

  delay(LOOP_DELAY);
}
