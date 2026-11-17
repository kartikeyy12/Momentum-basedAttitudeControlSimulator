/*
  ADCS-1 firmware v0.5.2-demo  -  Arduino Nano (ATmega328P). Libraries: Wire, EEPROM (both built in).
  Single-axis reaction-wheel pointing. Drive it from the Serial Monitor: 115200 baud, line ending "Newline".
  USB telemetry only. The nRF24 link, Kalman filter and friction feedforward are NOT in this version.

  v0.5-demo = v0.3 + two fixes found in closed-loop simulation (every v0.3 command, default and safety check is unchanged):
   1. ANTI-WINDUP in the hold law. Bearing friction holds the platform a little short of the target. v0.3 kept integrating that standing
      error into the wheel command until the wheel sat at its limit: after one move the platform could no longer be corrected in one direction
      (a flick one way never recovered). Now, when the platform is nearly still (< STUCK_RATE) and within STUCK_BAND of the target, the wheel
      command is no longer integrated and is bled slowly towards zero (a push far too weak to break the bearing loose), so the wheel keeps
      its headroom. A target less than STUCK_BAND away from where the platform sits therefore does not move it; use steps of 10 deg or more.
      'u' switches this off (exact v0.3 behaviour), for a platform that turns almost without friction.
   2. KEEP-ALIVE. Sending an empty line (just Enter) refreshes the 120 s no-command timer, so a long talk in HOLD does not stop the wheel.
  v0.5.1 adds print-only readouts (no control change): 's' shows maxrate (the largest platform spin rate the wheel can cancel, deg/s) and
  'i' ends with a GO / NO-GO line for the demo. Thresholds come from simulation with a guessed plant, treat them as a guide.
  v0.5.2 (MPU6050 mounted UPSIDE DOWN + an unbalanced, wire-laden platform):
   - GYRO SIGN. Flipped upside down, the sensor's Z axis points down, so counter-clockwise (seen from above) reads negative. GYRO_SIGN_DEFAULT
     is now -1 so that +angle = counter-clockwise again. The sign is saved in EEPROM together with gs, and 'j' flips it without re-flashing
     (it also flips gs, since gs is measured in the same coordinates). The control loop is sign-agnostic once 'i' has run, so a wrong guess
     only mirrors the direction of t<deg>; it cannot make the loop unstable. Old EEPROM records are ignored (new magic), so a gs identified
     in another orientation can never be reused: run m and i again after flashing.
   - 'v<deg>' sets the hold "stuck band" (default 3). Lower it if the platform creeps away from the target while idle in HOLD, raise it if
     the 'wheel saturated' note appears while the platform sits still.
   - 'd' = LEAN TEST (wheel off, 10 s): reports how far the platform moves by itself. Use it to level the base / balance the load.
     Software cannot cancel a steady torque from a leaning platform or stiff wires: the wheel only stores momentum, so that is a mechanical fix.

  HARDWARE ASSUMED (change the constants below if yours differ)
    MPU6050   I2C A4/A5, address 0x68, chip face up, Z axis vertical. Positive angle = counter-clockwise seen from above.
    Driver    two PWM inputs: IN1 = D5, IN2 = D6 (DRV8833 / DRV8871 / L9110 style). Tie SLEEP/EN high if the module has it.
    Battery   divider 100k (top) / 47k (bottom) midpoint to A0. Add 100 nF from A0 to GND for a stable reading.
    The motor has NO encoder, so wheel speed is never measured, only commanded (PWM ~ wheel speed).

  CONTROL (the repo's PD + 2-DOF prefilter design, rebuilt for this hardware)
    desired angular acceleration  a = wn^2 (r - angle) - 2 zeta wn rate
    r = target through the prefilter F(s) = z/(s+z), z = Kp/Kd = wn/(2 zeta)  (cancels the PD zero, no overshoot)
    The wheel is a speed actuator: platform rate = gs * (wheel command p), so an acceleration needs p' = a / gs.
    gs (deg/s per count, signed) and the motor start threshold are measured by 'm' then 'i'; the sign is learned too.
    p is integrated, clamped to +-pLimit (that clamp is the wheel's speed limit = saturation) and mapped past the dead zone.
    Repo reference: Kp = 0.01633 N m/rad, Kd = 0.008 N m s/rad = wn 5.6 rad/s, zeta 1.4 at the printed design's J_b = 5.17e-4
    ('w5.6' and 'y1.4' reproduce that). The defaults here (wn 10, zeta 1.3) are stiffer on purpose: the bearing's friction
    leaves a stuck error of roughly friction / (J_b * wn^2), so more stiffness means a smaller error.

  SAFETY
    Motor is OFF at boot. Send 'a' (arm) only with the shield fitted.
    x = soft stop (wheel spins down over about a second so it does not kick the platform), X = instant coast.
    Any fault, timeout or runaway disarms. Nothing here replaces the fuse.
*/
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>
#include <stdlib.h>

// ------------------------------ pins
const uint8_t PIN_IN1  = 5;
const uint8_t PIN_IN2  = 6;
const uint8_t PIN_VBAT = A0;
const uint8_t MPU_ADDR = 0x68;

// ------------------------------ fixed settings
const float GYRO_SIGN_DEFAULT = -1.0f;   // MPU6050 mounted UPSIDE DOWN (component side down): Z points down. Use +1.0f if it is mounted face up.
const float GYRO_LSB   = 65.5f;          // +-500 dps range
const unsigned long LOOP_US = 5000UL;    // 200 Hz
const float P_SLEW_PER_S    = 800.0f;    // max wheel-command change while controlling, counts/s
const float P_DOWN_PER_S    = 60.0f;     // soft-stop spin-down rate, counts/s
const float P_DZ            = 3.0f;      // counts either side of zero where the wheel stays still
const float DEAD_FRAC       = 0.8f;      // use 80 % of the measured start threshold (too high causes kicks, too low only a small lag)
const float RATE_ABORT      = 250.0f;    // deg/s, runaway guard
const float ERR_ABORT       = 120.0f;    // deg, runaway guard
const float STUCK_BAND_DEFAULT = 3.0f;   // deg: inside this band of the target and nearly still, the hold law stops integrating (anti-windup); 'v' changes it
const float STUCK_RATE      = 2.0f;      // deg/s: "nearly still"
const float BLEED_ACC       = 10.0f;     // deg/s^2: equivalent platform acceleration of the wheel-command bleed while stuck (about 15 % of the bearing's holding torque in the model)
const unsigned long CMD_TIMEOUT_MS = 120000UL;   // stop if no command for this long
const float VBAT_DIV   = (100.0f + 47.0f) / 47.0f;
const float VBAT_MIN   = 6.2f;           // 2 cells; the cutoff is OFF until you send 'b'
const float ID_P1      = 14.0f;          // identify pulse sizes, counts above the measured start threshold
const float ID_P2      = 40.0f;
const float ID_MIN_GS  = 0.02f;          // deg/s per count; below this the wheel/gyro did not respond

// ------------------------------ tunables (runtime)
float wn = 10.0f, zeta = 1.3f;
float gyroSign = GYRO_SIGN_DEFAULT;      // runtime copy, saved with gs, 'j' flips it
float stuckBand = STUCK_BAND_DEFAULT;    // runtime copy, 'v' changes it
uint8_t leanSt = 0;                      // lean test state (0 = off)
unsigned long leanT0 = 0, leanLast = 0;
float leanA0 = 0;
float pLimit = 180.0f;                   // wheel command limit, counts past the dead zone
bool  antiWindup = true;                 // 'u' toggles the v0.5 anti-windup (off = exact v0.3 hold law)
float aLim = 500.0f;                     // max commanded platform acceleration, deg/s^2 (keeps the wheel command from running ahead of the motor)
float dead = 0.0f;                       // PWM counts used as the motor start threshold
float deadDetect = 0.0f;                 // PWM counts where the platform first reacted ('m'), an upper bound
float gs = 0.0f;                         // deg/s of platform rate per wheel-command count, signed, 0 = unknown
bool  vbatCutoff = false;

// ------------------------------ state
const uint8_t M_IDLE = 0, M_OPEN = 1, M_HOLD = 2, M_RAMP = 3, M_IDENT = 4;
uint8_t mode = M_IDLE;
bool armed = false, stream = false, satShown = false, vbatLow = false, gyroFailShown = false;
unsigned long modeStart = 0, lastUs = 0, lastPrint = 0, lastCmdMs = 0, lowSince = 0, satSince = 0, lastVb = 0;
float gyroBias = 0, angle = 0, rate = 0, fr = 0, targetDeg = 0, rf = 0, vbat = 0;
float pState = 0, pOut = 0, openP = 0;
// 'm' and 'i' scratch
float idBase = 0, idSum = 0, idPk = 0, rampHit = 0, rampPk = 0;
float idKick[4] = {0, 0, 0, 0};
uint8_t idN = 0, idJ = 255, rampCnt = 0;
unsigned long rampK = 0;
bool rampEval = false;
// serial line buffer
char cmdBuf[16];
uint8_t cmdLen = 0;
unsigned long lastCharMs = 0;

struct Cfg { uint16_t magic; float gs; float dead; float deadDetect; int8_t sgn; };   // sgn: gyro sign that gs was identified with
const uint16_t CFG_MAGIC = 0xAD54;   // changed from 0xAD53 on purpose: older records (other orientation or supply) are ignored

// ------------------------------ motor
void setMotor(int16_t o) {
  if (o > 255) o = 255;
  if (o < -255) o = -255;
  if (o >= 0) { analogWrite(PIN_IN1, o);  analogWrite(PIN_IN2, 0);  }
  else        { analogWrite(PIN_IN1, 0);  analogWrite(PIN_IN2, -o); }
}

// wheel command p (counts, signed) -> PWM. Continuous: still inside +-P_DZ, then start threshold + (|p| - P_DZ).
int16_t wheelOut(float p) {
  float a = fabsf(p);
  float m = (a <= P_DZ) ? dead * a / P_DZ : dead + (a - P_DZ);
  if (m > 255.0f) m = 255.0f;
  int16_t o = (int16_t)(m + 0.5f);
  return (p >= 0) ? o : (int16_t)(-o);
}

// soft stop: disarm and let the main loop spin the wheel down gently (a sudden stop hands the wheel's momentum to the platform)
void stopAll(const char* why) {
  if (mode == M_RAMP || mode == M_IDENT) { pOut = 0; setMotor(0); }   // sharp-pulse modes, nothing to wind down
  mode = M_IDLE; armed = false; pState = 0;
  Serial.print(F("STOP: ")); Serial.print(why); Serial.println(F("  (disarmed)"));
}

void hardStop(const char* why) {
  stopAll(why);
  pOut = 0; setMotor(0);
}

void windDown() {                                  // blocking, at most a few seconds
  int guard = 0;
  while (pOut != 0.0f && guard < 800) {
    float st = P_DOWN_PER_S * 0.005f;
    if (pOut > st) pOut -= st; else if (pOut < -st) pOut += st; else pOut = 0.0f;
    setMotor(pOut == 0.0f ? 0 : wheelOut(pOut));
    delay(5);
    guard++;
  }
  setMotor(0); pOut = 0;
}

// ------------------------------ config in EEPROM
void saveCfg() {
  Cfg c; c.magic = CFG_MAGIC; c.gs = gs; c.dead = dead; c.deadDetect = deadDetect; c.sgn = (gyroSign < 0.0f) ? -1 : 1;
  EEPROM.put(0, c);
}

void loadCfg() {
  Cfg c; EEPROM.get(0, c);
  if (c.magic != CFG_MAGIC) return;
  if (isnan(c.gs) || isinf(c.gs) || isnan(c.dead) || isinf(c.dead) || isnan(c.deadDetect) || isinf(c.deadDetect)) return;
  if (c.dead < 0 || c.dead > 200 || c.deadDetect < 0 || c.deadDetect > 200) return;
  if (c.gs != 0 && (fabsf(c.gs) < 0.01f || fabsf(c.gs) > 20.0f)) return;
  if (c.sgn != 1 && c.sgn != -1) return;
  gs = c.gs; dead = c.dead; deadDetect = c.deadDetect; gyroSign = (float)c.sgn;
  Serial.print(F("Loaded from EEPROM: gs=")); Serial.print(gs, 4);
  Serial.print(F(" dead=")); Serial.print(dead, 0);
  Serial.print(F(" detect=")); Serial.print(deadDetect, 0);
  Serial.print(F(" sign=")); Serial.println((int)gyroSign);
}

// ------------------------------ MPU6050
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readGyroZ(int16_t &raw) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write((uint8_t)0x47);                       // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  raw = (int16_t)(((uint16_t)hi << 8) | lo);
  return true;
}

void calibrate() {
  pState = 0; windDown();
  Serial.println(F("Calibrating gyro, keep the platform still..."));
  long sum = 0; int n = 0; int16_t lo = 32767, hi = -32768;
  for (int i = 0; i < 400; i++) {
    int16_t r;
    if (readGyroZ(r)) { sum += r; n++; if (r < lo) lo = r; if (r > hi) hi = r; }
    delay(3);
  }
  if (n == 0) { Serial.println(F("MPU read failed, check wiring")); return; }
  gyroBias = (float)sum / (float)n / GYRO_LSB;
  float spread = (float)(hi - lo) / GYRO_LSB;
  Serial.print(F("bias dps = ")); Serial.print(gyroBias, 3);
  Serial.print(F("  spread dps = ")); Serial.println(spread, 2);
  if (spread > 3.0f) Serial.println(F("WARNING: platform moved during calibration, send z again"));
  angle = 0; rf = 0; targetDeg = 0; fr = 0; lastUs = micros();
}

// ------------------------------ battery
void updateVbat() {
  analogRead(PIN_VBAT);                            // first read settles the mux
  vbat = analogRead(PIN_VBAT) * 5.0f / 1023.0f * VBAT_DIV;
}

// ------------------------------ UI
void printHeader() { Serial.println(F("t_ms,angle_deg,rate_dps,ref_deg,p")); }

void printHelp() {
  Serial.println(F("h help | s status | g stream on/off | z zero angle + recalibrate gyro (keep still)"));
  Serial.println(F("a ARM (shield on!) | x soft stop | X instant coast | b battery cutoff on/off"));
  Serial.println(F("FIRST TIME: z, a, m (find motor start), a, i (identify), a, c (hold), t<deg> (target)"));
  Serial.println(F("o<p> open loop wheel command | t<deg> target | w<rad/s> wn | y<zeta> | l<counts> wheel limit"));
  Serial.println(F("n<counts> motor start threshold | q<deg/s per count> set gs by hand | r<deg/s^2> accel limit"));
  Serial.println(F("u anti-windup on/off (default ON) | Enter on an empty line = keep-alive"));
  Serial.println(F("j flip gyro sign (MPU upside down) | d lean test, wheel off | v<deg> hold stuck band"));
}

void printStatus() {
  Serial.print(F("armed=")); Serial.print(armed ? 1 : 0);
  Serial.print(F(" mode=")); Serial.print(mode);
  Serial.print(F(" gs=")); Serial.print(gs, 4);
  Serial.print(F(" dead=")); Serial.print(dead, 0);
  Serial.print(F(" wn=")); Serial.print(wn, 2);
  Serial.print(F(" zeta=")); Serial.print(zeta, 2);
  Serial.print(F(" limit=")); Serial.print(pLimit, 0);
  Serial.print(F(" maxrate=")); Serial.print(fabsf(gs) * pLimit, 1);
  Serial.print(F(" accel=")); Serial.print(aLim, 0);
  Serial.print(F(" target=")); Serial.print(targetDeg, 1);
  Serial.print(F(" vbat=")); Serial.print(vbat, 2);
  Serial.print(F(" cutoff=")); Serial.print(vbatCutoff ? 1 : 0);
  Serial.print(F(" sign=")); Serial.print((int)gyroSign);
  Serial.print(F(" band=")); Serial.println(stuckBand, 1);
}

bool needArmed() {
  if (!armed) { Serial.println(F("Not armed. Fit the shield, then send a")); return true; }
  return false;
}

void startMode(uint8_t m) {
  windDown();
  mode = m; modeStart = millis(); stream = true;
  pState = 0; pOut = 0; satShown = false; satSince = 0;
  lastUs = micros();
  printHeader();
}

void cmdSetFloat(float v, bool hasArg, float lo, float hi, float* dst, const char* name, bool save) {
  if (!hasArg || v < lo || v > hi) {
    Serial.print(name); Serial.print(F(" needs a value from ")); Serial.print(lo, 2); Serial.print(F(" to ")); Serial.println(hi, 2);
    return;
  }
  *dst = v;
  if (save) saveCfg();
  Serial.print(name); Serial.print(F("=")); Serial.println(v, 4);
}

void execCmd(const char* s) {
  char c = s[0];
  bool hasArg = (s[1] != 0);
  float v = hasArg ? (float)atof(s + 1) : 0.0f;
  lastCmdMs = millis();
  switch (c) {
    case 'h': printHelp(); break;
    case 's': printStatus(); break;
    case 'g':
      stream = !stream;
      if (stream) printHeader();
      break;
    case 'z':
      if (mode != M_IDLE) stopAll("recalibrate");
      calibrate();
      break;
    case 'a':
      armed = true;
      Serial.println(F("ARMED: shield on, hands clear"));
      break;
    case 'b':
      vbatCutoff = !vbatCutoff;
      Serial.print(F("battery cutoff "));
      Serial.println(vbatCutoff ? F("ON") : F("OFF"));
      break;
    case 'm':
      if (needArmed()) break;
      rampHit = 0; rampCnt = 0; rampK = 0; rampPk = 0; rampEval = false;
      startMode(M_RAMP);
      Serial.println(F("Pulsing the wheel with growing steps: it stops by itself when the platform reacts. Keep clear."));
      break;
    case 'i':
      if (needArmed()) break;
      if (deadDetect <= 0.0f) { Serial.println(F("Run m first (it finds the motor start threshold)")); break; }
      idN = 0; idJ = 255; idSum = 0; idBase = 0; idPk = 0;
      startMode(M_IDENT);
      Serial.println(F("Identifying (3.5 s): the platform will kick back and forth. Keep it clear."));
      break;
    case 'c':
      if (needArmed()) break;
      if (gs == 0.0f) { Serial.println(F("gs unknown: run i first (or set q)")); break; }
      startMode(M_HOLD);
      targetDeg = angle; rf = angle;
      Serial.println(F("HOLD on. t<deg> sets a target, x stops."));
      break;
    case 'o':
      if (!hasArg) { Serial.println(F("o needs a number, e.g. o30")); break; }
      if (needArmed()) break;
      openP = constrain(v, -pLimit, pLimit);
      startMode(M_OPEN);
      break;
    case 't':
      if (!hasArg) { Serial.println(F("t needs degrees, e.g. t30")); break; }
      targetDeg = constrain(v, -180.0f, 180.0f);
      Serial.print(F("target=")); Serial.println(targetDeg, 1);
      break;
    case 'w': cmdSetFloat(v, hasArg, 0.5f, 14.0f, &wn, "wn", false); break;
    case 'y': cmdSetFloat(v, hasArg, 0.3f, 3.0f, &zeta, "zeta", false); break;
    case 'l': cmdSetFloat(v, hasArg, 10.0f, 220.0f, &pLimit, "limit", false); break;
    case 'r': cmdSetFloat(v, hasArg, 50.0f, 3000.0f, &aLim, "accel limit", false); break;
    case 'n': cmdSetFloat(v, hasArg, 0.0f, 200.0f, &dead, "dead", true); break;
    case 'j':
      if (armed || mode != M_IDLE) { Serial.println(F("disarm first (x)")); break; }
      gyroSign = -gyroSign;
      if (gs != 0.0f) gs = -gs;
      saveCfg();
      Serial.print(F("gyro sign now ")); Serial.print((int)gyroSign);
      Serial.println(F(". Send z, then turn the platform counter-clockwise (seen from above): the angle must go UP."));
      break;
    case 'v': cmdSetFloat(v, hasArg, 1.0f, 6.0f, &stuckBand, "stuck band", false); break;
    case 'd':
      if (armed || mode != M_IDLE) { Serial.println(F("lean test: disarm first (x)")); break; }
      if (pOut != 0.0f) { Serial.println(F("wheel still winding down, wait 3 s")); break; }
      leanSt = 1; leanT0 = millis(); leanLast = leanT0; leanA0 = angle;
      Serial.println(F("LEAN TEST, 10 s, wheel off: let go of the platform and do not touch it."));
      break;
    case 'u':
      antiWindup = hasArg ? (v != 0.0f) : !antiWindup;
      Serial.print(F("anti-windup ")); Serial.println(antiWindup ? F("ON") : F("OFF (v0.3 behaviour)"));
      break;
    case 'q':
      if (hasArg && fabsf(v) >= 0.01f && fabsf(v) <= 20.0f) { gs = v; saveCfg(); Serial.print(F("gs=")); Serial.println(gs, 4); }
      else Serial.println(F("q needs 0.01 to 20 (signed)"));
      break;
    default:
      Serial.println(F("unknown command, h for help"));
      break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    lastCharMs = millis();
    if (c == '\n' || c == '\r') {
      if (cmdLen) { cmdBuf[cmdLen] = 0; cmdLen = 0; execCmd(cmdBuf); }
      else lastCmdMs = millis();                       // v0.5: an empty line (Enter) is a keep-alive for the 120 s timeout
      continue;
    }
    if (cmdLen == 0 && c == 'x') { stopAll("manual stop"); lastCmdMs = millis(); continue; }
    if (cmdLen == 0 && c == 'X') { hardStop("manual hard stop"); lastCmdMs = millis(); continue; }
    if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = c;
  }
  if (cmdLen && (millis() - lastCharMs) > 200UL) { cmdBuf[cmdLen] = 0; cmdLen = 0; execCmd(cmdBuf); }
}

// ------------------------------ per-mode controllers (each sets pState, the wheel command)
void runRamp(unsigned long t) {
  // Alternating +/- pulses of growing size: 4, 8, 12 ... PWM counts, 120 ms on, 280 ms off.
  // A slow ramp would never move the platform (bearing friction beats the tiny acceleration torque), so each level is a sharp step.
  // The first level where the platform kicks, twice in a row, is taken as the start threshold (an upper bound: friction hides small kicks).
  unsigned long k = t / 400UL;
  unsigned long ph = t % 400UL;
  float level = 4.0f * (float)(k + 1);
  if (level > 160.0f) { stopAll("m: no reaction up to 160, check driver, motor and gyro"); return; }
  float sgn = (k % 2UL == 0UL) ? 1.0f : -1.0f;
  if (ph < 120UL) {
    pState = sgn * level;                          // raw PWM counts in this mode
    if (rampK != k) { rampK = k; rampPk = 0; rampEval = false; }
    if (ph >= 15UL && fabsf(fr) > rampPk) rampPk = fabsf(fr);
  } else {
    pState = 0;
    if (!rampEval) {
      rampEval = true;
      if (rampPk > 0.5f) {
        if (rampCnt == 0) rampHit = level;
        rampCnt++;
      } else rampCnt = 0;
      if (rampCnt >= 2) {
        deadDetect = rampHit - 2.0f; if (deadDetect < 0) deadDetect = 0;
        dead = DEAD_FRAC * deadDetect;
        saveCfg();
        Serial.print(F("Platform reacted from PWM ")); Serial.print(rampHit, 0);
        Serial.print(F(", start threshold set to ")); Serial.println(dead, 0);
        stopAll("m done");
      }
    }
  }
}

void runIdent(unsigned long t) {
  // Four sharp pulses: +P1, -P1, +P2, -P2 (PWM = measured start threshold + size), 250 ms on, 450 ms rest each.
  // For a wheel-speed actuator kick = G * (pulse_pwm - true_start), so two sizes give
  //   G = slope (deg/s per count, signed) -> gs      and      the intercept = the true start threshold.
  if (t < 500UL) { pState = 0; return; }
  unsigned long u = t - 500UL;
  uint8_t j = (uint8_t)(u / 700UL);
  unsigned long ph = u % 700UL;
  if (j < 4) {
    float size = (j < 2) ? ID_P1 : ID_P2;
    float sgn  = (j % 2 == 0) ? 1.0f : -1.0f;
    if (j != idJ) { idJ = j; idSum = 0; idN = 0; idPk = 0; }
    if (ph < 60UL) { pState = 0; idSum += rate; idN++; }
    else {
      if (idN) { idBase = idSum / (float)idN; idN = 0; }
      if (ph < 310UL) {
        pState = sgn * (deadDetect + size);        // raw PWM counts in this mode
        if (ph >= 70UL) { float d = fr - idBase; if (fabsf(d) > fabsf(idPk)) idPk = d; }
      } else pState = 0;
      idKick[j] = idPk;
    }
    return;
  }
  pState = 0;
  float k1 = 0.5f * (idKick[0] - idKick[1]);       // kick for +P1 (antisymmetric average)
  float k2 = 0.5f * (idKick[2] - idKick[3]);
  Serial.print(F("kicks deg/s: +P1 ")); Serial.print(idKick[0], 2); Serial.print(F("  -P1 ")); Serial.print(idKick[1], 2);
  Serial.print(F("  +P2 ")); Serial.print(idKick[2], 2); Serial.print(F("  -P2 ")); Serial.println(idKick[3], 2);
  if (fabsf(k1) < 0.3f && fabsf(k2) < 0.3f) { stopAll("identify: no response. Run m, check driver, motor and gyro"); return; }
  if ((idKick[0] * idKick[1] >= 0.0f) || (idKick[2] * idKick[3] >= 0.0f)) { stopAll("identify: + and - pulses disagree, was the platform touched? try i again"); return; }
  float r1 = fabsf(idKick[0] / idKick[1]), r2 = fabsf(idKick[2] / idKick[3]);
  if (r1 < 0.4f || r1 > 2.5f || r2 < 0.4f || r2 > 2.5f) { stopAll("identify: + and - pulses differ a lot, try i again"); return; }
  if (k2 * k1 <= 0.0f || fabsf(k2) < 1.15f * fabsf(k1)) { stopAll("identify: the bigger pulse did not kick harder, check the motor"); return; }
  float G = (k2 - k1) / (ID_P2 - ID_P1);
  if (fabsf(G) < ID_MIN_GS) { stopAll("identify: gain too small"); return; }
  float dfit = deadDetect + ID_P1 - k1 / G;        // fitted true start threshold, PWM counts
  if (dfit < 0.0f) dfit = 0.0f;
  if (dfit > deadDetect) dfit = deadDetect;        // friction can only make the 'm' result too HIGH
  gs = G; dead = DEAD_FRAC * dfit;
  saveCfg();
  Serial.print(F("gs = ")); Serial.print(gs, 4);
  Serial.print(F(" deg/s per count, fitted start = ")); Serial.print(dfit, 0);
  Serial.print(F(", dead used = ")); Serial.print(dead, 0);
  Serial.print(F(". Max platform rate at the wheel limit = "));
  Serial.print(fabsf(gs) * pLimit, 1); Serial.println(F(" deg/s"));
  {
    float mr = fabsf(gs) * pLimit;
    if (mr >= 45.0f)      Serial.println(F("DEMO CHECK: GO. The wheel is strong enough for 10 deg steps and gentle flicks."));
    else if (mr >= 25.0f) Serial.println(F("DEMO CHECK: MARGINAL. Expect small steps and only partial recovery. More rim mass or a faster motor would help."));
    else                  Serial.println(F("DEMO CHECK: NO-GO. The wheel is too weak to turn the platform. Needs a faster motor or a heavier rim."));
  }
  stopAll("identify done");
}

void runHold(float dt) {
  float zf = wn / (2.0f * zeta);                   // prefilter pole = PD zero
  rf += zf * (targetDeg - rf) * dt;
  float e = rf - angle;
  if (fabsf(e) > ERR_ABORT) { stopAll("runaway: error too large"); return; }
  float a = wn * wn * e - 2.0f * zeta * wn * rate;          // desired angular acceleration, deg/s^2
  if (a > aLim) a = aLim;
  if (a < -aLim) a = -aLim;
  float pd = a / gs;                                         // wheel command rate, counts/s
  bool stuck = antiWindup && (fabsf(targetDeg - angle) < stuckBand) && (fabsf(fr) < STUCK_RATE);
  if (stuck) {                                               // v0.5: friction holds the platform; integrating would only wind the wheel up
    float bl = (BLEED_ACC / fabsf(gs)) * dt;                 // counts this tick
    if (pState > bl) pState -= bl; else if (pState < -bl) pState += bl; else pState = 0.0f;
  } else {
    bool pushingOut = (fabsf(pState) >= pLimit) && (pState * pd > 0.0f);
    if (!pushingOut) pState += pd * dt;
  }
  if (pState > pLimit) pState = pLimit;
  if (pState < -pLimit) pState = -pLimit;
  if (fabsf(pState) >= pLimit - 0.5f) {
    if (satSince == 0) satSince = millis();
    if (!satShown && (millis() - satSince) > 500UL) { Serial.println(F("NOTE: wheel saturated (at its limit)")); satShown = true; }
  } else { satSince = 0; satShown = false; }
}

// ------------------------------ setup / loop
void setup() {
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  setMotor(0);
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(400000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000, true);                 // a stuck I2C bus must not freeze the loop with the motor running
#endif
  mpuWrite(0x6B, 0x00);   // wake
  mpuWrite(0x1A, 0x03);   // low-pass filter ~44 Hz
  mpuWrite(0x1B, 0x08);   // gyro +-500 dps
  Serial.println(F("ADCS-1 firmware v0.5.2-demo. WARNING: never run the wheel without the shield."));
  loadCfg();
  calibrate();
  updateVbat();
  printStatus();
  printHelp();
  lastUs = micros(); lastCmdMs = millis();
}

void leanTick() {
  if (leanSt == 0) return;
  if (armed || mode != M_IDLE) { leanSt = 0; Serial.println(F("lean test cancelled")); return; }
  unsigned long now = millis();
  if ((now - leanLast) >= 2000UL) {
    leanLast = now;
    Serial.print(F("lean ")); Serial.print((now - leanT0) / 1000UL); Serial.print(F(" s, angle ")); Serial.println(angle, 1);
  }
  if ((now - leanT0) >= 10000UL) {
    leanSt = 0;
    float d = angle - leanA0;
    Serial.print(F("LEAN RESULT: moved ")); Serial.print(d, 1); Serial.print(F(" deg. "));
    if (fabsf(d) < 2.0f) Serial.println(F("BALANCED: friction holds it. Good."));
    else if (fabsf(d) < 8.0f) Serial.println(F("SLIGHT LEAN: shim the low side of the base level, or add weight opposite the heavy side."));
    else Serial.println(F("STRONG LEAN: it moves by itself. Level the base, balance it, put the wire bundle on the rotation axis."));
  }
}

void loop() {
  pollSerial();
  unsigned long nowU = micros();
  if ((nowU - lastUs) < LOOP_US) return;
  float dt = (float)(nowU - lastUs) * 1e-6f;
  lastUs = nowU;
  if (dt > 0.05f) dt = 0.05f;

  int16_t raw;
  if (!readGyroZ(raw)) {                           // no sensor, no control: kill the motor, say so once
    if (!gyroFailShown) { gyroFailShown = true; hardStop("gyro read failed"); }
    else { pOut = 0; setMotor(0); }
    return;
  }
  gyroFailShown = false;
  rate = gyroSign * ((float)raw / GYRO_LSB - gyroBias);
  angle += rate * dt;
  fr += 0.4f * (rate - fr);                        // lightly filtered rate, used for detection only
  leanTick();

  if (millis() - lastVb > 250UL) { lastVb = millis(); updateVbat(); }

  if (mode != M_IDLE) {
    if ((millis() - lastCmdMs) > CMD_TIMEOUT_MS) { stopAll("no command for 120 s"); return; }
    if (fabsf(rate) > RATE_ABORT)                { stopAll("runaway: rate too high"); return; }
    if (vbatCutoff && vbat > 1.0f && vbat < VBAT_MIN) {
      if (!vbatLow) { vbatLow = true; lowSince = millis(); }
      if ((millis() - lowSince) > 1000UL) { vbatLow = false; stopAll("low battery"); return; }
    } else vbatLow = false;
  }

  unsigned long t = millis() - modeStart;
  bool active = armed && (mode != M_IDLE);
  if (active) {
    if (mode == M_OPEN)       pState = openP;
    else if (mode == M_RAMP)  runRamp(t);
    else if (mode == M_IDENT) runIdent(t);
    else if (mode == M_HOLD)  runHold(dt);
  }
  if (!armed || mode == M_IDLE) pState = 0;

  // apply: sharp pulses unslewed (gentle ones cannot beat bearing friction), control slew-limited, stop = slow spin-down
  bool pulse = (mode == M_RAMP || mode == M_IDENT) && armed;
  float step = (active ? P_SLEW_PER_S : P_DOWN_PER_S) * dt;
  if (pulse) pOut = pState;
  else if (pState > pOut + step) pOut += step;
  else if (pState < pOut - step) pOut -= step;
  else pOut = pState;
  if (pulse)             setMotor((int16_t)pOut);  // raw PWM counts
  else if (pOut == 0.0f) setMotor(0);
  else                   setMotor(wheelOut(pOut));

  if (stream && (millis() - lastPrint) >= 50UL) {
    lastPrint = millis();
    Serial.print(millis());       Serial.print(',');
    Serial.print(angle, 2);       Serial.print(',');
    Serial.print(rate, 2);        Serial.print(',');
    Serial.print(rf, 2);          Serial.print(',');
    Serial.println(pOut, 1);
  }
}
