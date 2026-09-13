/* =============================================================================
 *  BADMINTON SHUTTLE LAUNCHER - ESP32 Dev Module firmware
 *
 *  !!!!!!!!!!!!!!!!!!!!!!!!!!!!  SAFETY FIRST  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *  * BENCH-TEST WITH THE WHEELS CLEAR. Clamp the frame down, keep hands, hair,
 *    cables and loose clothing out of the wheel path, and wear eye protection.
 *    Launch wheels will take skin off at idle throttle, never mind at firing
 *    throttle.
 *  * CALIBRATE THE ESC THROTTLE RANGE BEFORE FIRST REAL USE. An uncalibrated
 *    BLHeli ESC may read this firmware's 1000us "idle/arm" pulse as partial
 *    throttle and spin up the instant it is powered. Run the built-in `cal`
 *    command (see COMMANDS below) with the launch wheels removed, once per ESC
 *    set, before trusting anything else here.
 *  * Motors boot DISARMED and stay at the 1000us arming pulse until you type
 *    `arm`. Nothing spins before that.
 *  * `x` stops continuous fire and drops to idle. `disarm` cuts to 1000us and
 *    aborts any shot in progress. Keep the serial monitor focused and within
 *    reach while testing - it is the only stop button this thing has.
 *  * Power the servos and the ESC BEC separately from the ESP32 3V3 rail, and
 *    tie all grounds together. The ESP32 cannot source HS-422 stall current.
 *
 * -----------------------------------------------------------------------------
 *  HARDWARE
 *    ESC A (Parallax 20A OPTO BLHeli) signal ...... GPIO12
 *    ESC B (Parallax 20A OPTO BLHeli) signal ...... GPIO14
 *    Catch / hold servo (HS-422) .................. GPIO27
 *    Push / feed servo  (HS-422) .................. GPIO26
 *    Common ground between ESP32, ESCs and servo supply is REQUIRED.
 *
 *    NOTE on GPIO12: it is a strapping pin (MTDI) and must read LOW at reset on
 *    most ESP32 dev modules. An OPTO ESC signal input is a high-impedance
 *    opto/buffer input, so it normally floats low and boots fine. If the board
 *    ever refuses to boot with the ESC connected, add a 10k pulldown on GPIO12
 *    or move ESC A to another free output (e.g. GPIO13) and update PIN_ESC_A.
 *
 *  LIBRARY: ESP32Servo (Kevin Harrington / John K. Bennett). Install via the
 *           Arduino Library Manager. Board: "ESP32 Dev Module".
 *
 * -----------------------------------------------------------------------------
 *  LAUNCH CYCLE (one shot)
 *    0. ramp both ESCs idle -> fire throttle, wait MOTOR_SPINUP_MS
 *    1. catch servo -> RELEASE: drops the bottom shuttle, holds the stack
 *    2. dwell DWELL_AFTER_RELEASE_MS so the shuttle settles on the feed ramp
 *    3. push servo -> EXTEND: strokes the shuttle into the wheels
 *    4. push servo -> RETRACT
 *    5. catch servo -> HOLD: re-captures the stack, ready for the next shot
 *    6. hold fire throttle MOTOR_HOLD_AFTER_SHOT_MS, then drop back to idle
 *
 *  COMMANDS (115200 baud, newline-terminated)
 *    arm | a        arm: hold 1000us for ESC_ARM_HOLD_MS, then go to idle
 *    disarm | d     disarm: abort any shot, motors to 1000us
 *    f              fire one shot
 *    s N            set fire throttle. N <= 100 is percent of 1000-2000us,
 *                   N >= 1000 is raw microseconds. e.g. `s 45` or `s 1450`
 *    i N            set inter-shot interval in ms (continuous mode)
 *    r              run continuous at the set interval
 *    x              stop continuous, drop motors to idle (also aborts `cal`)
 *    jc N           jog catch servo to N degrees (bench tuning)
 *    jp N           jog push servo to N degrees (bench tuning)
 *    cal            guided ESC throttle-range calibration (disarmed only)
 *    k              "continue" / confirm step during `cal`
 *    p | ?          print status + the command list
 *
 *  TUNING: every value you are likely to change lives in the CONFIG block
 *  below, and each one carries a note on how to dial it in on the bench.
 * ===========================================================================*/

#include <ESP32Servo.h>
#include <ctype.h>    // tolower
#include <math.h>     // lround
#include <stdlib.h>   // atof, atol
#include <string.h>   // strcmp, strpbrk

/* ============================ CONFIG - PINS ============================== */

#define PIN_ESC_A         12   // ESC #1 signal
#define PIN_ESC_B         14   // ESC #2 signal
#define PIN_SERVO_CATCH   27   // HS-422 catch/hold servo
#define PIN_SERVO_PUSH    26   // HS-422 push/feed servo

/* ======================= CONFIG - ESCs AND MOTORS ========================= */

// Standard hobby servo-PWM band. Do not change unless the ESCs were calibrated
// to a different range.
static const int ESC_REFRESH_HZ = 50;    // 50 Hz frame, i.e. 20 ms period
static const int ESC_MIN_US     = 1000;  // full stop / arming pulse
static const int ESC_MAX_US     = 2000;  // full throttle

// Pulse held while DISARMED and during the arming hold. Must match the low end
// the ESCs were calibrated to, or they will not arm.
static const int MOTOR_ARM_US = 1000;

// How long to hold MOTOR_ARM_US before the ESCs are considered armed. BLHeli
// wants a steady low signal for a couple of seconds and answers with its arming
// tones. 3 s is the spec here; lengthen it if the ESCs are slow to chirp.
static const uint32_t ESC_ARM_HOLD_MS = 3000;

// TUNE: idle throttle held between shots. The wheels must keep turning but must
// NOT carry a shuttle. Start at 1100 and creep up in 10 us steps until both
// wheels spin reliably from cold without stuttering. Too low = one wheel stalls
// and the ESC desyncs; too high = shuttles creep through on their own.
static const int MOTOR_IDLE_US = 1120;

// TUNE: firing throttle = shot power. This is the boot default; change it live
// with `s N`. 1450 us is a conservative starting point - work up in 25 us steps
// and watch where the shuttle lands. Write the final value back here.
static const int MOTOR_FIRE_US_DEFAULT = 1450;

// Safety ceiling for `s N`. Anything above this is clamped. Raise deliberately,
// and only after the frame, wheels and mounts have proven themselves.
static const int MOTOR_FIRE_US_MAX = 1700;

// TUNE: time from "start ramping to fire throttle" to "release the shuttle".
// Too short and the wheels are still accelerating, so shots fall short and
// scatter. Fire ten shots: if the first ones land consistently shorter than the
// rest, increase this. 600-1000 ms is typical for loaded launch wheels.
static const uint32_t MOTOR_SPINUP_MS = 800;

// Ramp granularity: how often the throttle is nudged while spinning up. Ramping
// instead of hard-stepping is gentler on the ESCs and the battery.
static const uint32_t MOTOR_RAMP_STEP_MS = 20;

// TUNE: keep fire throttle this long AFTER the shuttle has been fed, so the
// wheels do not decelerate mid-shot. Increase if shots feel inconsistent at
// short intervals.
static const uint32_t MOTOR_HOLD_AFTER_SHOT_MS = 250;

/* ===================== CONFIG - SERVOS (HS-422) =========================== */

// HS-422 mechanical pulse range. The datasheet band is roughly 553-2270 us;
// 500-2500 gives full travel but can buzz at the extremes. If a servo hums at
// an end stop, narrow these before narrowing the angles below.
static const int SERVO_REFRESH_HZ = 50;   // analogue servos want 50 Hz
static const int SERVO_MIN_US     = 500;  // pulse written for write(0)
static const int SERVO_MAX_US     = 2500; // pulse written for write(180)

// TUNE (catch/hold servo, GPIO27): the finger/gate that supports the stack.
// HOLD    = gate under the bottom shuttle, whole stack supported.
// RELEASE = gate withdrawn just far enough to drop ONE shuttle while the next
//           one up is still pinched. Find both with `jc N` before trusting them.
static const int CATCH_HOLD_DEG    = 95;
static const int CATCH_RELEASE_DEG = 60;

// TUNE (push/feed servo, GPIO26): the arm that strokes the shuttle in.
// RETRACT = fully clear of the feed path, nothing fouls the next drop.
// EXTEND  = shuttle pushed far enough for the wheels to grab it, with the arm
//           stopping SHORT of the wheel nip. Set EXTEND with `jp N` while the
//           motors are DISARMED, and leave a few degrees of margin.
static const int PUSH_RETRACT_DEG = 20;
static const int PUSH_EXTEND_DEG  = 105;

/* ==================== CONFIG - CYCLE TIMING (ms) ========================== */

// TUNE: let the released shuttle settle on the ramp before pushing it. Too
// short and the push arm catches it mid-fall (jams/tumbles); too long just
// slows the rate of fire. Watch it in slow motion on a phone camera.
static const uint32_t DWELL_AFTER_RELEASE_MS = 150;

// TUNE: travel time allowed for the push stroke. An HS-422 does ~60 deg in
// 0.16 s at 6 V, so allow at least (degrees/60)*160 ms plus margin. If the
// stroke is still moving when the retract command lands, increase this.
static const uint32_t PUSH_EXTEND_MS = 250;

// TUNE: pause at full extension - long enough for the wheels to take the
// shuttle off the arm. Keep it short; this is dead time in the cycle.
static const uint32_t DWELL_AT_EXTEND_MS = 60;

// TUNE: travel time allowed for the retract stroke. Same arithmetic as above.
static const uint32_t PUSH_RETRACT_MS = 250;

// TUNE: small settle before the catch gate closes again, so the gate is not
// fighting a push arm that is still coming home.
static const uint32_t DWELL_BEFORE_REHOLD_MS = 40;

// TUNE: travel time for the catch gate to re-capture the stack.
static const uint32_t CATCH_REHOLD_MS = 150;

// TUNE: continuous-fire interval, measured shot-start to shot-start. If it is
// shorter than the cycle itself (~2 s with the values above) the launcher just
// fires back to back. Change it live with `i N`.
static const uint32_t SHOT_INTERVAL_MS_DEFAULT = 2500;

/* ============================ END OF CONFIG =============================== */

Servo escA;
Servo escB;
Servo servoCatch;
Servo servoPush;

enum MotorState : uint8_t {
  MS_DISARMED,   // holding MOTOR_ARM_US, nothing may spin
  MS_ARMING,     // holding MOTOR_ARM_US for ESC_ARM_HOLD_MS
  MS_ARMED       // idling at MOTOR_IDLE_US, ready to fire
};

enum ShotPhase : uint8_t {
  PH_IDLE,
  PH_SPINUP,        // ramp idle -> fire throttle
  PH_RELEASE,       // catch gate opened, shuttle settling
  PH_PUSH_OUT,      // push arm travelling to EXTEND
  PH_PUSH_HOLD,     // dwell at EXTEND
  PH_PUSH_BACK,     // push arm travelling to RETRACT
  PH_PRE_REHOLD,    // settle before the gate closes
  PH_REHOLD,        // catch gate travelling to HOLD
  PH_RECOVER        // hold fire throttle, then back to idle
};

enum CalStage : uint8_t {
  CAL_OFF,
  CAL_READY,    // signal at MIN, waiting for "ESC power is off" confirmation
  CAL_AT_MAX,   // signal at MAX, waiting for the user to power the ESCs
  CAL_AT_MIN    // signal at MIN, waiting for the confirmation tones
};

static MotorState motorState = MS_DISARMED;
static ShotPhase  shotPhase  = PH_IDLE;
static CalStage   calStage   = CAL_OFF;

static int      currentThrottleUs = MOTOR_ARM_US;
static int      fireThrottleUs    = MOTOR_FIRE_US_DEFAULT;
static uint32_t shotIntervalMs    = SHOT_INTERVAL_MS_DEFAULT;

static bool     continuousMode = false;
static uint32_t nextShotAtMs   = 0;

static uint32_t armStartMs   = 0;
static uint32_t phaseStartMs = 0;
static uint32_t lastRampMs   = 0;
static int      rampFromUs   = MOTOR_IDLE_US;

static int      catchAngleDeg = CATCH_HOLD_DEG;
static int      pushAngleDeg  = PUSH_RETRACT_DEG;

static char     cmdBuf[48];
static uint8_t  cmdLen = 0;

/* --------------------------- low-level outputs ---------------------------- */

static void writeMotors(int us) {
  if (us < ESC_MIN_US) us = ESC_MIN_US;
  if (us > ESC_MAX_US) us = ESC_MAX_US;
  currentThrottleUs = us;
  escA.writeMicroseconds(us);
  escB.writeMicroseconds(us);   // both ESCs always get the identical pulse
}

static void writeCatch(int deg) {
  if (deg < 0)   deg = 0;
  if (deg > 180) deg = 180;
  catchAngleDeg = deg;
  servoCatch.write(deg);
}

static void writePush(int deg) {
  if (deg < 0)   deg = 0;
  if (deg > 180) deg = 180;
  pushAngleDeg = deg;
  servoPush.write(deg);
}

/* ------------------------------- helpers ---------------------------------- */

static const char *motorStateName() {
  switch (motorState) {
    case MS_DISARMED: return "DISARMED";
    case MS_ARMING:   return "ARMING";
    default:          return "ARMED";
  }
}

static const char *shotPhaseName() {
  switch (shotPhase) {
    case PH_IDLE:       return "idle";
    case PH_SPINUP:     return "spin-up";
    case PH_RELEASE:    return "release";
    case PH_PUSH_OUT:   return "push-out";
    case PH_PUSH_HOLD:  return "push-hold";
    case PH_PUSH_BACK:  return "push-back";
    case PH_PRE_REHOLD: return "pre-rehold";
    case PH_REHOLD:     return "rehold";
    default:            return "recover";
  }
}

static int usToPercent(int us) {
  return (int)lround((us - ESC_MIN_US) * 100.0 / (ESC_MAX_US - ESC_MIN_US));
}

static void printHelp() {
  Serial.println(F("commands:"));
  Serial.println(F("  arm | a     arm ESCs (1000us hold, then idle)"));
  Serial.println(F("  disarm | d  disarm, abort shot, motors to 1000us"));
  Serial.println(F("  f           fire one shot"));
  Serial.println(F("  s N         fire throttle: N<=100 = percent, N>=1000 = us"));
  Serial.println(F("  i N         inter-shot interval, ms"));
  Serial.println(F("  r           run continuous"));
  Serial.println(F("  x           stop continuous / abort cal, motors to idle"));
  Serial.println(F("  jc N        jog catch servo to N deg"));
  Serial.println(F("  jp N        jog push servo to N deg"));
  Serial.println(F("  cal         guided ESC throttle-range calibration"));
  Serial.println(F("  k           continue/confirm during cal"));
  Serial.println(F("  p | ?       this status"));
}

static void printStatus() {
  Serial.println(F("---- status ----"));
  Serial.printf("  motors      : %s, output %d us\n", motorStateName(), currentThrottleUs);
  Serial.printf("  fire thr    : %d us (%d%%)\n", fireThrottleUs, usToPercent(fireThrottleUs));
  Serial.printf("  idle thr    : %d us\n", MOTOR_IDLE_US);
  Serial.printf("  shot phase  : %s\n", shotPhaseName());
  Serial.printf("  continuous  : %s, interval %lu ms\n",
                continuousMode ? "ON" : "off", (unsigned long)shotIntervalMs);
  Serial.printf("  catch servo : %d deg (hold %d / release %d)\n",
                catchAngleDeg, CATCH_HOLD_DEG, CATCH_RELEASE_DEG);
  Serial.printf("  push servo  : %d deg (retract %d / extend %d)\n",
                pushAngleDeg, PUSH_RETRACT_DEG, PUSH_EXTEND_DEG);
  if (calStage != CAL_OFF) Serial.println(F("  *** ESC CALIBRATION IN PROGRESS ***"));
  printHelp();
}

/* ---------------------------- shot state machine -------------------------- */

// Entry actions for each phase live here so the sequence reads top to bottom.
// Everything is non-blocking, so serial (and therefore `x` / `disarm`) stays
// responsive in the middle of a shot.
static void enterPhase(ShotPhase p) {
  shotPhase    = p;
  phaseStartMs = millis();

  switch (p) {
    case PH_SPINUP:
      rampFromUs = currentThrottleUs;
      lastRampMs = phaseStartMs;
      break;
    case PH_RELEASE:   writeCatch(CATCH_RELEASE_DEG); break;  // step 1
    case PH_PUSH_OUT:  writePush(PUSH_EXTEND_DEG);    break;  // step 3
    case PH_PUSH_BACK: writePush(PUSH_RETRACT_DEG);   break;  // step 4
    case PH_REHOLD:    writeCatch(CATCH_HOLD_DEG);    break;  // step 5
    case PH_PUSH_HOLD:                                        // dwell only
    case PH_PRE_REHOLD:                                       // dwell only
    case PH_RECOVER:                                          // step 6
    case PH_IDLE:
    default: break;
  }
}

static bool startShot(bool quiet) {
  if (motorState != MS_ARMED) {
    if (!quiet) Serial.println(F("[err] motors not armed - send `arm` first"));
    return false;
  }
  if (shotPhase != PH_IDLE) {
    if (!quiet) Serial.println(F("[err] shot already in progress"));
    return false;
  }
  if (calStage != CAL_OFF) {
    if (!quiet) Serial.println(F("[err] ESC calibration in progress - send `x` first"));
    return false;
  }
  Serial.printf("[shot] firing at %d us (%d%%)\n", fireThrottleUs, usToPercent(fireThrottleUs));
  enterPhase(PH_SPINUP);
  return true;
}

// Put the mechanism back into a safe, known state without touching arm state.
static void abortShot() {
  if (shotPhase == PH_IDLE) return;
  writePush(PUSH_RETRACT_DEG);
  writeCatch(CATCH_HOLD_DEG);
  shotPhase = PH_IDLE;
  Serial.println(F("[shot] aborted - servos returned to hold/retract"));
}

static void updateShot() {
  if (shotPhase == PH_IDLE) return;
  const uint32_t now = millis();
  const uint32_t t   = now - phaseStartMs;

  switch (shotPhase) {
    case PH_SPINUP:
      // Linear ramp idle -> fire throttle across MOTOR_SPINUP_MS.
      if (MOTOR_SPINUP_MS == 0) {
        writeMotors(fireThrottleUs);
        enterPhase(PH_RELEASE);
      } else {
        if (now - lastRampMs >= MOTOR_RAMP_STEP_MS) {
          lastRampMs = now;
          const uint32_t span = (t > MOTOR_SPINUP_MS) ? MOTOR_SPINUP_MS : t;
          writeMotors(rampFromUs +
                      (int)(((long)(fireThrottleUs - rampFromUs) * (long)span) /
                            (long)MOTOR_SPINUP_MS));
        }
        if (t >= MOTOR_SPINUP_MS) {
          writeMotors(fireThrottleUs);
          enterPhase(PH_RELEASE);
        }
      }
      break;

    case PH_RELEASE:
      if (t >= DWELL_AFTER_RELEASE_MS) enterPhase(PH_PUSH_OUT);
      break;

    case PH_PUSH_OUT:
      if (t >= PUSH_EXTEND_MS) enterPhase(PH_PUSH_HOLD);
      break;

    case PH_PUSH_HOLD:
      if (t >= DWELL_AT_EXTEND_MS) enterPhase(PH_PUSH_BACK);
      break;

    case PH_PUSH_BACK:
      if (t >= PUSH_RETRACT_MS) enterPhase(PH_PRE_REHOLD);
      break;

    case PH_PRE_REHOLD:
      if (t >= DWELL_BEFORE_REHOLD_MS) enterPhase(PH_REHOLD);
      break;

    case PH_REHOLD:
      if (t >= CATCH_REHOLD_MS) enterPhase(PH_RECOVER);
      break;

    case PH_RECOVER:
      if (t >= MOTOR_HOLD_AFTER_SHOT_MS) {
        writeMotors(MOTOR_IDLE_US);   // back to idle - the ESCs are never stopped
        shotPhase = PH_IDLE;
        Serial.println(F("[shot] complete - motors at idle"));
      }
      break;

    default:
      shotPhase = PH_IDLE;
      break;
  }
}

/* ------------------------------ arm / disarm ------------------------------ */

static void cmdArm() {
  if (calStage != CAL_OFF) {
    Serial.println(F("[err] finish or abort calibration (`x`) before arming"));
    return;
  }
  if (motorState == MS_ARMED)  { Serial.println(F("[arm] already armed"));      return; }
  if (motorState == MS_ARMING) { Serial.println(F("[arm] already arming..."));  return; }

  writeCatch(CATCH_HOLD_DEG);
  writePush(PUSH_RETRACT_DEG);
  writeMotors(MOTOR_ARM_US);
  motorState = MS_ARMING;
  armStartMs = millis();
  Serial.printf("[arm] holding %d us for %lu ms - KEEP CLEAR OF THE WHEELS\n",
                MOTOR_ARM_US, (unsigned long)ESC_ARM_HOLD_MS);
}

static void cmdDisarm() {
  continuousMode = false;
  abortShot();
  writeMotors(MOTOR_ARM_US);
  motorState = MS_DISARMED;
  Serial.printf("[arm] DISARMED - output %d us\n", MOTOR_ARM_US);
}

static void updateArming() {
  if (motorState != MS_ARMING) return;
  if (millis() - armStartMs >= ESC_ARM_HOLD_MS) {
    motorState = MS_ARMED;
    writeMotors(MOTOR_IDLE_US);
    Serial.printf("[arm] ARMED - idling at %d us. `f` to fire, `x` to stop.\n", MOTOR_IDLE_US);
  }
}

/* --------------------------- continuous fire ------------------------------ */

static void updateContinuous() {
  if (!continuousMode) return;

  if (motorState != MS_ARMED) {        // disarmed out from under us
    continuousMode = false;
    Serial.println(F("[run] stopped - motors not armed"));
    return;
  }
  if (shotPhase != PH_IDLE) return;    // still working on the previous shot

  if ((int32_t)(millis() - nextShotAtMs) >= 0) {
    nextShotAtMs = millis() + shotIntervalMs;   // interval is shot-start to shot-start
    startShot(true);
  }
}

/* ------------------------- ESC range calibration -------------------------- */

static void calAbort(const char *why) {
  calStage = CAL_OFF;
  writeMotors(MOTOR_ARM_US);
  motorState = MS_DISARMED;
  Serial.printf("[cal] aborted (%s) - output back to %d us\n", why, MOTOR_ARM_US);
}

static void cmdCal() {
  if (motorState != MS_DISARMED) {
    Serial.println(F("[err] `disarm` before calibrating"));
    return;
  }
  if (calStage != CAL_OFF) {
    Serial.println(F("[cal] already running - `k` to continue, `x` to abort"));
    return;
  }

  continuousMode = false;
  calStage = CAL_READY;
  writeMotors(MOTOR_ARM_US);
  Serial.println(F("[cal] ESC THROTTLE RANGE CALIBRATION"));
  Serial.println(F("[cal] Remove the launch wheels (or make certain nothing can be"));
  Serial.println(F("[cal] hit) and DISCONNECT ESC BATTERY POWER now."));
  Serial.println(F("[cal] Send `k` once ESC power is disconnected. `x` aborts."));
}

// Called when `k` arrives.
static void calContinue() {
  switch (calStage) {
    case CAL_READY:
      calStage = CAL_AT_MAX;
      writeMotors(ESC_MAX_US);
      Serial.printf("[cal] output is now FULL throttle (%d us).\n", ESC_MAX_US);
      Serial.println(F("[cal] Connect ESC battery power NOW. Wait for the ESCs to"));
      Serial.println(F("[cal] beep out the high-point tones, then send `k`."));
      break;

    case CAL_AT_MAX:
      calStage = CAL_AT_MIN;
      writeMotors(ESC_MIN_US);
      Serial.printf("[cal] output is now MINIMUM (%d us).\n", ESC_MIN_US);
      Serial.println(F("[cal] Wait for the confirmation tones, then send `k` to finish."));
      break;

    case CAL_AT_MIN:
      calStage = CAL_OFF;
      writeMotors(MOTOR_ARM_US);
      motorState = MS_DISARMED;
      Serial.println(F("[cal] done. Range stored in the ESCs. Power-cycle the ESCs,"));
      Serial.println(F("[cal] then `arm` and check that idle throttle behaves."));
      break;

    default:
      Serial.println(F("[err] `k` only means something during `cal`"));
      break;
  }
}

/* ----------------------------- serial commands ---------------------------- */

// `s N`: N in (0,100] is percent of the 1000-2000us band, N >= 1000 is raw us.
// (100 is read as 100%, not 100us.) Result is clamped to idle..MOTOR_FIRE_US_MAX.
static void cmdSetThrottle(const char *arg) {
  if (!arg || !*arg) {
    Serial.println(F("[err] usage: s <percent 1-100 | us 1000-2000>"));
    return;
  }
  const double v = atof(arg);
  int us;

  if (v > 0.0 && v <= 100.0) {
    us = ESC_MIN_US + (int)lround(v * (ESC_MAX_US - ESC_MIN_US) / 100.0);
  } else if (v >= (double)ESC_MIN_US && v <= (double)ESC_MAX_US) {
    us = (int)lround(v);
  } else {
    Serial.println(F("[err] out of range: use 1-100 (percent) or 1000-2000 (us)"));
    return;
  }

  if (us < MOTOR_IDLE_US) {
    Serial.printf("[warn] %d us is below idle - clamped to %d us\n", us, MOTOR_IDLE_US);
    us = MOTOR_IDLE_US;
  }
  if (us > MOTOR_FIRE_US_MAX) {
    Serial.printf("[warn] %d us exceeds MOTOR_FIRE_US_MAX - clamped to %d us\n",
                  us, MOTOR_FIRE_US_MAX);
    us = MOTOR_FIRE_US_MAX;
  }

  fireThrottleUs = us;
  Serial.printf("[set] fire throttle = %d us (%d%%)\n", fireThrottleUs, usToPercent(fireThrottleUs));
  // Takes effect on the next shot; a shot already spinning up keeps its target.
}

static void cmdSetInterval(const char *arg) {
  if (!arg || !*arg) { Serial.println(F("[err] usage: i <ms>")); return; }
  const long v = atol(arg);
  if (v < 100 || v > 600000L) { Serial.println(F("[err] interval must be 100-600000 ms")); return; }
  shotIntervalMs = (uint32_t)v;
  Serial.printf("[set] interval = %lu ms\n", (unsigned long)shotIntervalMs);
}

// Live servo jogging for bench tuning: note the angle that works, then copy it
// into the CONFIG block above so it survives a reset.
static void cmdJog(bool isCatch, const char *arg) {
  if (shotPhase != PH_IDLE) { Serial.println(F("[err] shot in progress - wait or `x`")); return; }
  if (!arg || !*arg) {
    Serial.printf("[err] usage: %s <0-180>\n", isCatch ? "jc" : "jp");
    return;
  }
  const long deg = atol(arg);
  if (deg < 0 || deg > 180) { Serial.println(F("[err] angle must be 0-180")); return; }
  if (motorState != MS_DISARMED) {
    // Jogging with the wheels turning can drop or feed a shuttle by hand.
    Serial.println(F("[jog] CAUTION: motors are not disarmed - the wheels are live"));
  }

  if (isCatch) { writeCatch((int)deg); Serial.printf("[jog] catch servo -> %d deg\n", catchAngleDeg); }
  else         { writePush((int)deg);  Serial.printf("[jog] push servo  -> %d deg\n", pushAngleDeg); }
}

static void cmdRun() {
  if (motorState != MS_ARMED) { Serial.println(F("[err] motors not armed - send `arm` first")); return; }
  if (calStage != CAL_OFF)    { Serial.println(F("[err] calibration in progress - send `x` first")); return; }
  continuousMode = true;
  nextShotAtMs   = millis();   // first shot immediately
  Serial.printf("[run] continuous ON, every %lu ms. `x` to stop.\n", (unsigned long)shotIntervalMs);
}

static void cmdStop() {
  if (calStage != CAL_OFF) { calAbort("user stop"); return; }
  continuousMode = false;
  abortShot();
  if (motorState == MS_ARMED) {
    writeMotors(MOTOR_IDLE_US);
    Serial.printf("[stop] continuous off, motors at idle (%d us)\n", MOTOR_IDLE_US);
  } else {
    Serial.println(F("[stop] continuous off"));
  }
}

static void handleCommand(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (!*line) return;

  for (char *p = line; *p; ++p) *p = (char)tolower((unsigned char)*p);

  char *arg = strpbrk(line, " \t");
  if (arg) { *arg++ = '\0'; while (*arg == ' ' || *arg == '\t') arg++; }

  if      (!strcmp(line, "f"))                            startShot(false);
  else if (!strcmp(line, "s"))                            cmdSetThrottle(arg);
  else if (!strcmp(line, "i"))                            cmdSetInterval(arg);
  else if (!strcmp(line, "r"))                            cmdRun();
  else if (!strcmp(line, "x"))                            cmdStop();
  else if (!strcmp(line, "arm")    || !strcmp(line, "a")) cmdArm();
  else if (!strcmp(line, "disarm") || !strcmp(line, "d")) cmdDisarm();
  else if (!strcmp(line, "jc"))                           cmdJog(true, arg);
  else if (!strcmp(line, "jp"))                           cmdJog(false, arg);
  else if (!strcmp(line, "cal"))                          cmdCal();
  else if (!strcmp(line, "k"))                            calContinue();
  else if (!strcmp(line, "p") || !strcmp(line, "?"))      printStatus();
  else Serial.printf("[err] unknown command '%s' - send `?` for help\n", line);
}

static void handleSerial() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) { cmdBuf[cmdLen] = '\0'; cmdLen = 0; handleCommand(cmdBuf); }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
    // Overlong lines simply stop accumulating; the newline still dispatches.
  }
}

/* --------------------------------- setup ---------------------------------- */

void setup() {
  Serial.begin(115200);
  delay(200);

  // ESP32Servo needs LEDC timers reserved before any attach().
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // ESCs first, so the low/arming pulse is present from the earliest moment.
  escA.setPeriodHertz(ESC_REFRESH_HZ);
  escB.setPeriodHertz(ESC_REFRESH_HZ);
  escA.attach(PIN_ESC_A, ESC_MIN_US, ESC_MAX_US);
  escB.attach(PIN_ESC_B, ESC_MIN_US, ESC_MAX_US);
  writeMotors(MOTOR_ARM_US);

  // Servos to their safe positions: stack held, feed path clear.
  servoCatch.setPeriodHertz(SERVO_REFRESH_HZ);
  servoPush.setPeriodHertz(SERVO_REFRESH_HZ);
  servoCatch.attach(PIN_SERVO_CATCH, SERVO_MIN_US, SERVO_MAX_US);
  servoPush.attach(PIN_SERVO_PUSH, SERVO_MIN_US, SERVO_MAX_US);
  writeCatch(CATCH_HOLD_DEG);
  writePush(PUSH_RETRACT_DEG);

  Serial.println();
  Serial.println(F("=== badminton shuttle launcher ==="));
  Serial.println(F("SAFETY: bench-test with the wheels clear. Calibrate the ESC"));
  Serial.println(F("        throttle range (`cal`) before first real use."));
  Serial.printf("[boot] holding %d us for %lu ms so the ESCs see a steady low signal...\n",
                MOTOR_ARM_US, (unsigned long)ESC_ARM_HOLD_MS);

  delay(ESC_ARM_HOLD_MS);   // ESC arming hold - nothing can spin during this

  // The ESCs now have their arming signal, but THIS FIRMWARE stays disarmed:
  // the output remains at MOTOR_ARM_US until the operator types `arm`.
  motorState = MS_DISARMED;
  writeMotors(MOTOR_ARM_US);

  Serial.printf("[boot] ready. Motors DISARMED at %d us - nothing spins until `arm`.\n",
                MOTOR_ARM_US);
  Serial.printf("[boot] fire throttle %d us (%d%%), interval %lu ms.\n",
                fireThrottleUs, usToPercent(fireThrottleUs), (unsigned long)shotIntervalMs);
  printHelp();
}

/* ---------------------------------- loop ----------------------------------- */

void loop() {
  handleSerial();      // stays responsive during every phase below
  updateArming();
  updateShot();
  updateContinuous();
}
