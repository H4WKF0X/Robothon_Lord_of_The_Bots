#include <MeAuriga.h>

// false = normal (hug RIGHT wall)
// true  = mirrored (hug LEFT wall)
#define MIRRORED_TRACK false

// ── Ports ─────────────────────────────────────────────────────────────────────
#define COLOR_PORT         PORT_7   // ← was wrongly PORT_8 before
#define US_RIGHT_PORT      PORT_8
#define US_LEFT_PORT       PORT_9
#define LINE_FOLLOWER_PORT PORT_6
#define MOTOR_LEFT_SLOT    SLOT_1
#define MOTOR_RIGHT_SLOT   SLOT_2

// ── Line follower ─────────────────────────────────────────────────────────────
const int BASE_SPEED = 70;
const int TURN_SOFT  = 50;
const int TURN_HARD  = 40;
const int LOOP_DELAY = 10;

// ── Tunnel ────────────────────────────────────────────────────────────────────
const int   TUNNEL_SPEED        = 65;
const float WALL_TARGET_CM      = 9.0f;  // desired distance to hugged wall
const float WALL_DEADBAND_CM    = 2.0f;  // tolerance before correcting
const int   WALL_CORRECTION     = 20;    // how hard to steer when off

const float MIDDLE_WALL_GONE_CM = 25.0f; // opposite US threshold for end-of-tunnel
const int   WALL_GONE_CONFIRM   = 5;     // consecutive reads needed

const int ENTRY_STRAIGHT_MS  = 1500;    // drive blind into tunnel before hugging
const int TURN90_OUTER_SPEED = 70;
const int TURN90_INNER_SPEED = 10;
const int TURN90_MS          = 480;     // tune until each corner ≈ 90°
const int STRAIGHT_CORNER_MS = 200;     // straight across closed end of U
const int EXIT_SPEED         = 55;
const int EXIT_TIMEOUT_MS    = 3000;

// Color sensor thresholds
const int RED_R_MIN = 80;
const int RED_G_MAX = 60;
const int RED_B_MAX = 60;

// ── Objects ───────────────────────────────────────────────────────────────────
MeLineFollower     lineSensor(LINE_FOLLOWER_PORT);
MeColorSensor      colorSensor(COLOR_PORT);
MeUltrasonicSensor usRight(US_RIGHT_PORT);
MeUltrasonicSensor usLeft(US_LEFT_PORT);
MeEncoderOnBoard   motorLeft(MOTOR_LEFT_SLOT);
MeEncoderOnBoard   motorRight(MOTOR_RIGHT_SLOT);

// ── State machine ─────────────────────────────────────────────────────────────
enum TunnelState {
  TUNNEL_IDLE,
  TUNNEL_ENTRY_STRAIGHT,  // drive blind until walls are in sensor range
  TUNNEL_HUG_GO,          // hug wall inward, watch for middle wall end
  TUNNEL_CORNER_1,        // first 90° arc
  TUNNEL_CORNER_MID,      // short straight across closed end of U
  TUNNEL_CORNER_2,        // second 90° arc → facing back out
  TUNNEL_HUG_RETURN,      // hug wall back out, watch color sensor
  TUNNEL_EXIT_FIND,       // creep forward until color = B/W
  TUNNEL_DONE
};

static TunnelState   tState      = TUNNEL_IDLE;
static unsigned long tStateStart = 0;
static int           wallGoneCnt = 0;
static bool          hugRight    = true;

static void enterState(TunnelState s) {
  tState      = s;
  tStateStart = millis();
}

// ── Motors ────────────────────────────────────────────────────────────────────
static void _setMotors(int l, int r) {
  motorLeft.setMotorPwm(-l);
  motorRight.setMotorPwm(r);
}
static void _stop()           { _setMotors(0, 0); }
static void _forward(int spd) { _setMotors(spd, spd); }

// ── Line follower helpers ─────────────────────────────────────────────────────
void goStraight()              { _setMotors(BASE_SPEED, BASE_SPEED); }
void turnRight(int correction) { _setMotors(BASE_SPEED, BASE_SPEED - correction); }
void turnLeft(int correction)  { _setMotors(BASE_SPEED - correction, BASE_SPEED); }

// ── Color sensor ──────────────────────────────────────────────────────────────
bool colorIsRed() {
  colorSensor.ColorDataRead();
  uint16_t r = colorSensor.ReturnRedData();
  uint16_t g = colorSensor.ReturnGreenData();
  uint16_t b = colorSensor.ReturnBlueData();
  return (r > RED_R_MIN && g < RED_G_MAX && b < RED_B_MAX);
}
bool colorIsLine() { return !colorIsRed(); }

// ── Wall helpers ──────────────────────────────────────────────────────────────
static float hugWallDist()      { return hugRight ? usRight.distanceCm() : usLeft.distanceCm(); }
static float oppositeWallDist() { return hugRight ? usLeft.distanceCm()  : usRight.distanceCm(); }

// ── Bang-bang wall hug ────────────────────────────────────────────────────────
static void doWallHug() {
  float dist = hugWallDist();
  if (dist < WALL_TARGET_CM - WALL_DEADBAND_CM) {
    // too close → steer away
    if (hugRight) _setMotors(TUNNEL_SPEED - WALL_CORRECTION, TUNNEL_SPEED + WALL_CORRECTION);
    else          _setMotors(TUNNEL_SPEED + WALL_CORRECTION, TUNNEL_SPEED - WALL_CORRECTION);
  } else if (dist > WALL_TARGET_CM + WALL_DEADBAND_CM) {
    // too far → steer toward
    if (hugRight) _setMotors(TUNNEL_SPEED + WALL_CORRECTION, TUNNEL_SPEED - WALL_CORRECTION);
    else          _setMotors(TUNNEL_SPEED - WALL_CORRECTION, TUNNEL_SPEED + WALL_CORRECTION);
  } else {
    _forward(TUNNEL_SPEED);
  }
}

// ── Tunnel API ────────────────────────────────────────────────────────────────
void tunnelBegin() {
  hugRight    = !MIRRORED_TRACK;
  wallGoneCnt = 0;
  enterState(TUNNEL_ENTRY_STRAIGHT);
}

void tunnelUpdate() {
  unsigned long elapsed = millis() - tStateStart;

  switch (tState) {

    case TUNNEL_ENTRY_STRAIGHT:
      _forward(TUNNEL_SPEED);
      if (elapsed >= ENTRY_STRAIGHT_MS) {
        _stop();
        delay(1000);  // pause so you can see where the robot ended up
        wallGoneCnt = 0;
        enterState(TUNNEL_HUG_GO);
      }
      break;

    case TUNNEL_HUG_GO:
      doWallHug();
      if (oppositeWallDist() > MIDDLE_WALL_GONE_CM) wallGoneCnt++;
      else                                           wallGoneCnt = 0;
      if (wallGoneCnt >= WALL_GONE_CONFIRM) {
        wallGoneCnt = 0;
        enterState(TUNNEL_CORNER_1);
      }
      break;

    case TUNNEL_CORNER_1:
      if (hugRight) _setMotors(TURN90_INNER_SPEED, TURN90_OUTER_SPEED);
      else          _setMotors(TURN90_OUTER_SPEED, TURN90_INNER_SPEED);
      if (elapsed >= TURN90_MS) enterState(TUNNEL_CORNER_MID);
      break;

    case TUNNEL_CORNER_MID:
      _forward(TUNNEL_SPEED);
      if (elapsed >= STRAIGHT_CORNER_MS) enterState(TUNNEL_CORNER_2);
      break;

    case TUNNEL_CORNER_2:
      if (hugRight) _setMotors(TURN90_INNER_SPEED, TURN90_OUTER_SPEED);
      else          _setMotors(TURN90_OUTER_SPEED, TURN90_INNER_SPEED);
      if (elapsed >= TURN90_MS) enterState(TUNNEL_HUG_RETURN);
      break;

    case TUNNEL_HUG_RETURN:
      doWallHug();
      if (colorIsLine()) {
        _stop();
        enterState(TUNNEL_EXIT_FIND);
      }
      break;

    case TUNNEL_EXIT_FIND:
      _forward(EXIT_SPEED);
      if (colorIsLine() || elapsed > EXIT_TIMEOUT_MS) {
        _stop();
        enterState(TUNNEL_DONE);
      }
      break;

    case TUNNEL_DONE:
      _stop();
      break;

    default:
      break;
  }
}

bool tunnelDone() { return tState == TUNNEL_DONE; }

// ── Line follower ─────────────────────────────────────────────────────────────
void followLine() {
  switch (lineSensor.readSensors()) {
    case S1_IN_S2_IN:   turnRight(TURN_SOFT); break;
    case S1_IN_S2_OUT:  goStraight();          break;
    case S1_OUT_S2_IN:  turnLeft(TURN_HARD);   break;
    case S1_OUT_S2_OUT: turnLeft(TURN_SOFT);   break;
    default:            _stop();               break;
  }
}

// ── Setup & loop ──────────────────────────────────────────────────────────────
static bool inTunnel   = false;
static int  redConfirm = 0;

void setup() {
  colorSensor.SensorInit();
  _stop();
  delay(2000);
}

void loop() {
  if (!inTunnel) {
    if (colorIsRed()) {
      _stop();  // stop immediately on first red detect
      redConfirm++;
      if (redConfirm >= 3) {
        redConfirm = 0;
        inTunnel   = true;
        delay(100);
        tunnelBegin();
      }
    } else {
      redConfirm = 0;
      followLine();
    }
  } else {
    tunnelUpdate();
    if (tunnelDone()) inTunnel = false;
  }
  delay(LOOP_DELAY);
}
