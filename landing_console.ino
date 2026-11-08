// landing_console.ino - the Arduino LANDING CONSOLE.
//   It receives the terrain results (from hazard_sw.py over USB, or from the FPGA wire on D2) and runs a landing sequence:
//     SEARCH    yellow blinks fast with a radar ping every second or so. The best spot is secured within a quarter of a second of being offered;
//               objects on the ground do NOT stop it. Only 'no safe spot anywhere' (laptop sends 255): red on, yellow flashing, warning beeps.
//     LOCKED    lock-on: green strobes with a rising 4-note chirp, then green steady with a slow yellow heartbeat. The servo arrow points at the
//               target. If a NEW target is sent and holds still for 0.25 s (for example the AI picked one) it re-locks onto it.
//               Lower your hand over the distance sensor (below 28 cm) to start the descent.
//     DESCEND   green and yellow chase each other, the buzzer ticks faster AND higher as your hand gets lower. Below 12 cm both flash together.
//     TOUCHDOWN hand below 7 cm with the target still safe: victory fanfare, green/yellow celebration strobe, then both steady.
//     ABORT     the target became a hazard (or the data stopped): police-style siren, red and yellow strobing in turn, the servo arrow shakes.
//               Raise your hand to reset.
//   After TOUCHDOWN or ABORT you must raise your hand above 30 cm before the next descent.
//
// WIRING (Arduino unplugged from USB while wiring; every GND is the Arduino GND):
//   Servo (pointer arm)  signal D9, red 5V, brown GND
//   LEDs, each through a 330 ohm resistor to the LONG leg, short leg to GND:   D7 green   D8 yellow   D10 red
//   Buzzer               + to D6, other leg to GND
//   Distance sensor HC-SR04 (the "altitude" sensor): VCC 5V, GND, TRIG D12, ECHO D13, pointing UP. Your hand is the lander.
//   Optional FPGA wire   FPGA serial pin -> D2 (and GND to GND)
//
// MESSAGES IN (9600 baud, over USB or D2):   Z<cell hex 2>,<hazard count hex 2>,<hazard map: 36 hex digits>
//   TEST WITHOUT THE DISTANCE SENSOR: type  D20  (then Enter) in the Serial Monitor: pretends your hand is at 20 cm for 2 seconds.
// MESSAGES OUT (USB, 9600 baud):  ST <STATE> ...   (status lines the laptop software shows),   OK column=...  (what it understood)
#include <Servo.h>
#include <SoftwareSerial.h>

const bool HAS_SERVO  = true;
const bool HAS_BUZZER = true;
const bool HAS_DIST   = true;           // set false if the HC-SR04 is not wired (then use the D20-style test commands)
const int NO_SITE_COUNT = 250;          // the laptop reports 255 when there is NO safe spot anywhere in the yard (objects everywhere)
const unsigned long LOCK_MS = 250;      // a safe landing spot is secured as soon as it has been the same for this long (2-3 packets)
const unsigned long RELOCK_MS = 250;    // a better spot is secured as quickly as that
const int START_DESCENT_CM = 28;        // hand lower than this starts the descent
const int TOUCH_CM = 7;                 // hand lower than this = touchdown
const int RAISE_CM = 30;                // hand higher than this re-arms the console
const int NO_HAND_CM = 40;              // a reading above this (or no echo) means "no hand over the sensor"
// distance sensor filtering: the HC-SR04 gives wrong or missing readings now and then, so nothing reacts to a single reading
const unsigned long PING_MS = 50;       // one ping every 50 ms; the MEDIAN of the last 5 pings is used (one or two bad pings are ignored)
const unsigned long HOLD_DROPOUT_MS = 400;   // if the echo disappears for less than this, keep the last good distance
const unsigned long DESCEND_HOLD_MS = 300;   // the hand must stay below START_DESCENT_CM this long before the descent starts
const unsigned long TOUCH_HOLD_MS = 200;     // ... and below TOUCH_CM this long for a touchdown
const unsigned long LEAVE_HOLD_MS = 700;     // the hand must be above START_DESCENT_CM + LEAVE_MARGIN_CM this long to cancel a descent
const int LEAVE_MARGIN_CM = 8;               // (so a hand hovering near the 28 cm line cannot flip the state back and forth)
const unsigned long RAISE_HOLD_MS = 400;     // re-arming needs the hand above RAISE_CM for this long
const byte ABORT_PACKETS = 2;                // the target must be reported as a hazard in this many packets in a row before an ABORT

const int FPGA_RX = 2, FPGA_TX_UNUSED = 3;
const int SERVO_PIN = 9, LED_GREEN = 7, LED_YELLOW = 8, LED_RED = 10, BUZZER_PIN = 6;
const int TRIG_PIN = 12, ECHO_PIN = 13;

SoftwareSerial fpga(FPGA_RX, FPGA_TX_UNUSED);
Servo arm;
char bufUsb[96];
byte lenUsb = 0;
char bufFpga[96];
byte lenFpga = 0;

// ---- terrain data ----
bool gotMsg = false;
unsigned long lastMsg = 0;
int curIdx = -1, curHaz = 0;
bool maskKnown = false;
uint8_t hazMask[18];                    // bit i = cell i is a hazard

// ---- landing state ----
enum State { SEARCH, LOCKED, DESCEND, TOUCHDOWN, ABORT };
State state = SEARCH;
unsigned long stateSince = 0;
int targetIdx = -1, stableIdx = -1;
unsigned long stableSince = 0;
bool needRaise = false;
int altCm = 99;                         // filtered distance, 99 = no hand
int samples[5] = {99, 99, 99, 99, 99};
byte sampPos = 0;
int lastGoodCm = 99;
byte goodRun = 0;
unsigned long lastGoodAt = 0;
struct Hold { bool on; unsigned long since; };
Hold hBelow = {false, 0}, hTouch = {false, 0}, hAway = {false, 0}, hRaise = {false, 0};
byte blockedPkts = 0;
int angleWritten = -1;
int fakeCm = -1;
unsigned long fakeUntil = 0, lastDist = 0, lastTick = 0, lastStatus = 0, lastEcho = 0;
int angleNow = 90, angleTarget = 90;
unsigned long lastStep = 0;

// ---- non-blocking sound ----
struct Note { int f; int ms; };
Note queueNotes[12];
byte qLen = 0, qPos = 0;
unsigned long noteEnd = 0;
int lastToneF = -1;                     // pitch of a continuous tone now playing (-1 none, -2 a queued note is playing)
unsigned long lastPing = 0;
unsigned int tickCount = 0;

void playSong(const Note* n, byte count) {
  for (byte i = 0; i < count && i < 12; i++) queueNotes[i] = n[i];
  qLen = count; qPos = 0; noteEnd = 0;
}
void soundTask(unsigned long now) {
  if (qPos < qLen && now >= noteEnd) {
    Note n = queueNotes[qPos++];
    if (HAS_BUZZER && n.f > 0) tone(BUZZER_PIN, n.f, n.ms);
    lastToneF = -2;
    noteEnd = now + n.ms;
  }
}
bool soundBusy(unsigned long now) { return qPos < qLen || now < noteEnd; }
void setTone(int f) {                    // a continuous tone (siren); only re-programs the buzzer when the pitch changes
  if (f == lastToneF) return;
  lastToneF = f;
  if (!HAS_BUZZER) return;
  if (f > 0) tone(BUZZER_PIN, f); else noTone(BUZZER_PIN);
}

const char* stateName(State s) {
  switch (s) { case SEARCH: return "SEARCH"; case LOCKED: return "LOCKED"; case DESCEND: return "DESCEND";
               case TOUCHDOWN: return "TOUCHDOWN"; default: return "ABORT"; }
}

void printStatus() {
  Serial.print("ST ");
  Serial.print(stateName(state));
  if (state != SEARCH) {
    Serial.print(" target=");
    Serial.print(targetIdx % 16);
    Serial.print(",");
    Serial.print(targetIdx / 16);
  }
  if (altCm < NO_HAND_CM) {
    Serial.print(" alt=");
    Serial.print(altCm);
  }
  Serial.println("");
}

void setState(State s) {
  state = s;
  blockedPkts = 0;
  stateSince = millis();
  lastTick = millis();
  tickCount = 0;
  if (s == LOCKED) { static const Note n[] = {{900, 70}, {1200, 70}, {1600, 70}, {2200, 230}}; playSong(n, 4); }
  if (s == TOUCHDOWN) {
    static const Note n[] = {{523, 110}, {659, 110}, {784, 110}, {1047, 260}, {0, 40}, {784, 110}, {1047, 110}, {1568, 480}};
    playSong(n, 8); needRaise = true;
  }
  if (s == ABORT) { playSong(0, 0); needRaise = true; }      // the siren itself is played by outputTask
  printStatus();
  lastStatus = millis();
}

int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

bool isHazard(int idx) {
  return idx >= 0 && idx < 144 && ((hazMask[idx >> 3] >> (idx & 7)) & 1);
}

// "Z11,05,<36 hex digits>"  ->  cell, hazard count, and (if present) the hazard map
bool parsePacket(const char* s, int* idx, int* haz, bool* hasMask) {
  if (s[0] != 'Z' || s[3] != ',' || s[6] != ',') return false;
  int a = hexVal(s[1]), b = hexVal(s[2]), c = hexVal(s[4]), d = hexVal(s[5]);
  if (a < 0 || b < 0 || c < 0 || d < 0) return false;
  *idx = a * 16 + b;
  *haz = c * 16 + d;
  *hasMask = false;
  const char* m = s + 7;
  int n = 0;
  while (n < 36 && hexVal(m[n]) >= 0) n++;
  if (n == 36) {
    uint8_t tmp[18];
    memset(tmp, 0, sizeof(tmp));
    for (int k = 0; k < 36; k++) {
      int v = hexVal(m[k]);
      int base = (35 - k) * 4;
      for (int bit = 0; bit < 4; bit++) if ((v >> bit) & 1) { int cell = base + bit; tmp[cell >> 3] |= (1 << (cell & 7)); }
    }
    memcpy(hazMask, tmp, sizeof(tmp));
    *hasMask = true;
  }
  return true;
}

void handleLine(const char* s) {
  if (s[0] == 'D') {                              // test command: D20 = pretend a hand is at 20 cm for 2 s
    fakeCm = atoi(s + 1);
    fakeUntil = millis() + 2000;
    return;
  }
  int idx, haz;
  bool hasMask;
  if (!parsePacket(s, &idx, &haz, &hasMask)) return;
  curIdx = idx;
  curHaz = haz;
  maskKnown = hasMask;
  if ((state == LOCKED || state == DESCEND) && targetIdx >= 0 && hasMask && isHazard(targetIdx)) { if (blockedPkts < 250) blockedPkts++; }
  else blockedPkts = 0;
  gotMsg = true;
  lastMsg = millis();
  if (millis() - lastEcho > 500) {
    lastEcho = millis();
    Serial.print("OK column=");
    Serial.print(idx % 16);
    Serial.print(" row=");
    Serial.print(idx / 16);
    Serial.print(" hazards=");
    Serial.print(haz);
    Serial.print(" rough=");
    Serial.println(haz >= NO_SITE_COUNT ? 1 : 0);
  }
}

void feed(Stream& in, char* buf, byte& len) {
  while (in.available()) {
    char c = in.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < 95) {
      buf[len++] = c;
    } else {
      len = 0;
    }
  }
}

long readCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long us = pulseIn(ECHO_PIN, HIGH, 4000UL);      // 4 ms = 68 cm: anything farther (ceiling, walls) simply counts as "no hand"
  if (us == 0) return -1;
  long cm = us / 58;
  return cm < 2 ? -1 : cm;                        // under 2 cm the sensor cannot measure: treat as a missed ping
}

int median5() {
  int t[5];
  for (byte i = 0; i < 5; i++) t[i] = samples[i];
  for (byte i = 1; i < 5; i++) { int v = t[i]; int j = i - 1; while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; } t[j + 1] = v; }
  return t[2];
}

void altitudeTask(unsigned long now) {
  if (now - lastDist < PING_MS) return;
  lastDist = now;
  if (fakeUntil > now) { altCm = fakeCm; return; }                 // test command: bypasses the sensor
  if (!HAS_DIST) { altCm = 99; return; }
  long cm = readCm();
  samples[sampPos] = (cm > 0 && cm < NO_HAND_CM) ? (int)cm : 99;   // a miss or a far echo is stored as 99
  sampPos = (sampPos + 1) % 5;
  int m = median5();                                               // one or two junk pings out of five change nothing
  if (m < 99) { if (goodRun < 250) goodRun++; lastGoodCm = m; lastGoodAt = now; altCm = m; }
  else if (goodRun >= 6 && now - lastGoodAt < HOLD_DROPOUT_MS) altCm = lastGoodCm;   // a hand that was really there (6+ good pings): ride out a short dropout
  else { altCm = 99; goodRun = 0; }                                // a brief false "hand" that vanishes at once is dropped at once
}

// true once `cond` has been true without a break for `ms` (call it on every pass, whatever the state)
bool held(Hold &h, bool cond, unsigned long ms, unsigned long now) {
  if (!cond) { h.on = false; return false; }
  if (!h.on) { h.on = true; h.since = now; }
  return now - h.since >= ms;
}

bool targetStillSafe() {
  if (maskKnown) return blockedPkts < ABORT_PACKETS;
  return curIdx == targetIdx;
}

void logicTask(unsigned long now) {
  bool linkAlive = gotMsg && (now - lastMsg) < 2000;
  bool noSite = curHaz >= NO_SITE_COUNT;
  // debounced hand conditions (updated on every pass)
  bool dropReady = held(hBelow, altCm <= START_DESCENT_CM, DESCEND_HOLD_MS, now);
  bool touched   = held(hTouch, altCm <= TOUCH_CM, TOUCH_HOLD_MS, now);
  bool handAway  = held(hAway, altCm >= START_DESCENT_CM + LEAVE_MARGIN_CM, LEAVE_HOLD_MS, now);
  bool raised    = held(hRaise, altCm >= RAISE_CM, RAISE_HOLD_MS, now);
  if (needRaise && raised) needRaise = false;
  if (!linkAlive) { if (state != SEARCH) setState(SEARCH); stableIdx = -1; return; }
  bool curOk = curIdx >= 0 && (!maskKnown || !isHazard(curIdx));     // the offered spot is a real safe spot
  switch (state) {
    case SEARCH:                                                     // secure the best spot as soon as one is offered: no need for smooth ground
      if (!noSite && curOk) {
        if (curIdx == stableIdx) {
          if (now - stableSince >= LOCK_MS) { targetIdx = curIdx; setState(LOCKED); }
        } else { stableIdx = curIdx; stableSince = now; }
      } else stableIdx = -1;
      break;
    case LOCKED:
      if (!targetStillSafe()) { setState(SEARCH); break; }
      if (curOk && curIdx != targetIdx) {                            // a better spot was found: re-secure it quickly
        if (curIdx == stableIdx) { if (now - stableSince >= RELOCK_MS) { targetIdx = curIdx; setState(LOCKED); break; } }
        else { stableIdx = curIdx; stableSince = now; }
      } else stableIdx = -1;
      if (!needRaise && dropReady) setState(DESCEND);
      break;
    case DESCEND:
      if (!targetStillSafe()) { setState(ABORT); break; }
      if (touched) { setState(TOUCHDOWN); break; }
      if (handAway) setState(LOCKED);
      break;
    case TOUCHDOWN:
      if (now - stateSince > 4000) setState(SEARCH);
      break;
    case ABORT:
      if (now - stateSince > 3000) setState(SEARCH);
      break;
  }
  if ((state == DESCEND || state == LOCKED) && now - lastStatus > 500) { lastStatus = now; printStatus(); }
}

void outputTask(unsigned long now) {
  bool linkAlive = gotMsg && (now - lastMsg) < 2000;
  bool rough = curHaz >= NO_SITE_COUNT;
  bool g = false, y = false, r = false;
  if (!linkAlive && lastToneF > 0) setTone(0);
  if (linkAlive) {
    unsigned long age = now - stateSince;
    switch (state) {
      case SEARCH:
        y = ((now / 150) % 2) == 0;                                         // fast scanning blink
        r = rough;
        if (!soundBusy(now)) {
          if (rough && now - lastPing > 900) { lastPing = now; static const Note w[] = {{330, 150}, {0, 60}, {330, 150}}; playSong(w, 3); }
          else if (!rough && now - lastPing > 1300) { lastPing = now; static const Note p[] = {{1760, 35}, {0, 40}, {2093, 35}}; playSong(p, 3); }   // radar ping
        }
        break;
      case LOCKED:
        g = (age < 450) ? (((now / 60) % 2) == 0) : true;                   // lock-on strobe, then steady
        y = (age >= 450) && ((now % 2000) < 70);                            // slow heartbeat
        break;
      case DESCEND: {
        int a = constrain(altCm, TOUCH_CM, START_DESCENT_CM);
        long interval = map(a, TOUCH_CM, START_DESCENT_CM, 90, 520);
        if (now - lastTick >= (unsigned long)interval) {
          lastTick = now; tickCount++;
          int f = map(a, TOUCH_CM, START_DESCENT_CM, 2600, 900);            // closer = higher pitch
          if (HAS_BUZZER) tone(BUZZER_PIN, f, 35);
          lastToneF = -2;
        }
        bool flash = (now - lastTick) < 90;
        bool finalApproach = altCm <= 12;
        g = flash && (finalApproach || (tickCount % 2 == 0));
        y = flash && (finalApproach || (tickCount % 2 == 1));
        break;
      }
      case TOUCHDOWN:
        if (age < 1600) { g = ((now / 100) % 2) == 0; y = !g; }             // celebration strobe
        else { g = true; y = true; }
        break;
      case ABORT:
        r = (now % 200) < 130;                                              // red and yellow strobe in turn
        y = !r;
        if (!soundBusy(now)) setTone((((now / 150) % 2) == 0) ? 980 : 640); // siren
        break;
    }
  }
  if (state != ABORT && lastToneF > 0) setTone(0);                          // siren off
  digitalWrite(LED_GREEN, g);
  digitalWrite(LED_YELLOW, y);
  digitalWrite(LED_RED, r);
  // the arrow points at the target column (or the current safe column while searching)
  int cell = (state != SEARCH && targetIdx >= 0) ? targetIdx : curIdx;
  angleTarget = (linkAlive && cell >= 0) ? (int)map(constrain(cell % 16, 0, 15), 0, 15, 20, 160) : 90;
  if (linkAlive && state == ABORT) angleTarget = constrain(angleTarget + ((((now / 110) % 2) == 0) ? 22 : -22), 0, 180);   // alarm shake
  if (now - lastStep > 15) {
    lastStep = now;
    if (angleNow < angleTarget) angleNow = min(angleNow + 3, angleTarget);
    else if (angleNow > angleTarget) angleNow = max(angleNow - 3, angleTarget);
    if (HAS_SERVO && angleNow != angleWritten) { arm.write(angleNow); angleWritten = angleNow; }
  }
}

void setup() {
  Serial.begin(9600);
  fpga.begin(9600);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  if (HAS_SERVO) { arm.attach(SERVO_PIN); arm.write(90); }
  const int chase[5] = {LED_GREEN, LED_YELLOW, LED_RED, LED_YELLOW, LED_GREEN};
  const int notes[5] = {523, 659, 784, 1047, 1319};
  for (int i = 0; i < 5; i++) { digitalWrite(chase[i], HIGH); if (HAS_BUZZER) tone(BUZZER_PIN, notes[i], 90); delay(100); digitalWrite(chase[i], LOW); }
  for (int k = 0; k < 2; k++) {                                              // all-LED flash
    digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_YELLOW, HIGH); digitalWrite(LED_RED, HIGH);
    if (HAS_BUZZER) tone(BUZZER_PIN, 2093, 70);
    delay(90);
    digitalWrite(LED_GREEN, LOW); digitalWrite(LED_YELLOW, LOW); digitalWrite(LED_RED, LOW);
    delay(70);
  }
  Serial.println("landing_console v2 ready: listening on USB and on D2");
}

void loop() {
  feed(Serial, bufUsb, lenUsb);
  feed(fpga, bufFpga, lenFpga);
  unsigned long now = millis();
  altitudeTask(now);
  logicTask(now);
  outputTask(now);
  soundTask(now);
  delay(2);
}
