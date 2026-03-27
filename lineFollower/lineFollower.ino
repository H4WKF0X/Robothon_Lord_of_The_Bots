/**
 * @file    lineFollower.ino
 * @brief   Modular line follower for mBot Ranger (Auriga board)
 *          Follows the boundary between a black and white surface.
 *
 * @hardware
 *   - Board  : mBot Ranger (MeAuriga)
 *   - Sensor : Me Line Follower on PORT_9
 *   - Motors : Left  encoder motor on SLOT_1
 *              Right encoder motor on SLOT_2
 *
 * @sensor states (black line = 0, white = 1)
 *   S1_IN_S2_IN   (0x00)  both sensors on black  → lost / on line
 *   S1_IN_S2_OUT  (0x01)  left on black, right on white → veer left
 *   S1_OUT_S2_IN  (0x02)  left on white, right on black → veer right
 *   S1_OUT_S2_OUT (0x03)  both on white → lost / on white
 *
 * @strategy
 *   The robot rides the LEFT edge of the black line:
 *     - S1(left)=black, S2(right)=white  →  perfectly on the edge, go straight
 *     - Both black                        →  drifted into black, turn right
 *     - Both white                        →  drifted out, turn left  (last-seen)
 *     - S1=white, S2=black               →  crossed to wrong side, correct hard
 */

#include <MeAuriga.h>

// ─────────────────────────────────────────────
//  CONFIGURATION — tweak these values freely
// ─────────────────────────────────────────────

// Sensor port (blue port on Auriga)
#define LINE_FOLLOWER_PORT  PORT_6

// Motor slots
#define MOTOR_LEFT_SLOT   SLOT_1
#define MOTOR_RIGHT_SLOT  SLOT_2

// Base speed  (0–255)
const int BASE_SPEED   = 150;

// Correction amounts
const int TURN_SOFT    = 60;   // gentle correction
const int TURN_HARD    = 120;  // aggressive correction when fully off track

// Loop delay in ms
const int LOOP_DELAY   = 10;

// ─────────────────────────────────────────────
//  HARDWARE OBJECTS
// ─────────────────────────────────────────────

MeLineFollower lineSensor(LINE_FOLLOWER_PORT);
MeEncoderOnBoard motorLeft(MOTOR_LEFT_SLOT);
MeEncoderOnBoard motorRight(MOTOR_RIGHT_SLOT);

// ─────────────────────────────────────────────
//  MOTOR HELPERS
// ─────────────────────────────────────────────

/**
 * Set both motors.
 * Positive speed = forward for that side.
 * Right motor is negated because it is mounted mirrored.
 */
void setMotors(int leftSpeed, int rightSpeed) {
  motorLeft.setMotorPwm(leftSpeed);
  motorRight.setMotorPwm(-rightSpeed);   // inverted mounting
}

void stopMotors() {
  setMotors(0, 0);
}

void goStraight() {
  setMotors(BASE_SPEED, BASE_SPEED);
}

/** Turn right: left motor faster, right motor slower */
void turnRight(int correction) {
  setMotors(BASE_SPEED, BASE_SPEED - correction);
}

/** Turn left: right motor faster, left motor slower */
void turnLeft(int correction) {
  setMotors(BASE_SPEED - correction, BASE_SPEED);
}

// ─────────────────────────────────────────────
//  LINE FOLLOWER LOGIC
// ─────────────────────────────────────────────

/**
 * Read the line sensor and drive accordingly.
 * The robot tracks the LEFT edge of a black line on white.
 */
void followLine() {
  int state = lineSensor.readSensors();

  switch (state) {

    // ── Both sensors on black: drifted INTO the line → correct rightward
    case S1_IN_S2_IN:
      turnRight(TURN_SOFT);
      break;

    // ── Left black, right white: perfect edge position → go straight
    case S1_IN_S2_OUT:
      goStraight();
      break;

    // ── Left white, right black: crossed to wrong side → correct hard left
    case S1_OUT_S2_IN:
      turnLeft(TURN_HARD);
      break;

    // ── Both white: drifted fully out of the line → search left (last resort)
    case S1_OUT_S2_OUT:
      turnLeft(TURN_SOFT);
      break;

    default:
      stopMotors();
      break;
  }
}

// ─────────────────────────────────────────────
//  DEBUG HELPER  (disable by commenting out)
// ─────────────────────────────────────────────

// #define DEBUG_SERIAL

void debugPrint(int state) {
#ifdef DEBUG_SERIAL
  switch (state) {
    case S1_IN_S2_IN:   Serial.println("Both BLACK  → turn right"); break;
    case S1_IN_S2_OUT:  Serial.println("Edge OK     → straight");   break;
    case S1_OUT_S2_IN:  Serial.println("Crossed     → turn left");  break;
    case S1_OUT_S2_OUT: Serial.println("Both WHITE  → search left");break;
  }
#endif
}

// ─────────────────────────────────────────────
//  ARDUINO ENTRY POINTS
// ─────────────────────────────────────────────

void setup() {
#ifdef DEBUG_SERIAL
  Serial.begin(9600);
  Serial.println("Line follower started (edge mode)");
#endif

  stopMotors();
  delay(2000);   // safety pause before moving
}

void loop() {
#ifdef DEBUG_SERIAL
  debugPrint(lineSensor.readSensors());
#endif

  followLine();
  delay(LOOP_DELAY);
}
