/*
 * Voltage Protector - Arduino Uno / Wokwi, simulation only.
 * A0: emulated 0..300 V; A1: emulated 0..10 A (not actual load sensing).
 * LCD: RS=7 E=6 D4=5 D5=4 D6=3 D7=2.
 * Buttons to GND: MENU=A2 PLUS=A3 MINUS=A4 RESET=A5.
 * Relay=8 (active LOW, diagram's default npn model); fault LED=9.
 * Settings live in RAM. Restart restores defaults. See readme.md.
 */
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/pgmspace.h>

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
const uint8_t RELAY_PIN = 8, FAULT_PIN = 9;
const uint8_t RELAY_ON = LOW, RELAY_OFF = HIGH;
const uint32_t SAMPLE_US = 2000, RECOVERY_MS = 2000;
const uint8_t BUTTON_PINS[4] = {A2, A3, A4, A5};
enum State : uint8_t { STARTUP, RUNNING, TRIPPED };
enum Fault : uint8_t { NONE = 0, UV = 1, OV = 2, OC = 4 };
State state = STARTUP;
uint16_t vMin = 180, vMax = 250; // volts
uint16_t iMax10 = 50;           // tenths of an ampere
uint16_t voltage10 = 0, current10 = 0;
uint8_t fault = NONE;
uint16_t tripVoltage10 = 0, tripCurrent10 = 0;
uint32_t tripAtMs = 0, tripCount = 0;
uint32_t safeSince = 0, lastSampleUs = 0, maxSampleGapUs = 0;
bool safeTiming = false, haveSample = false;
uint8_t menuPage = 0; // 0=status, 1=VMIN, 2=VMAX, 3=IMAX
uint16_t editValue = 0;
uint32_t lastLcdMs = 0, lastStreamMs = 0;
bool streamEnabled = false;
char desired[2][17] = {{0}, {0}}, shown[2][17] = {{0}, {0}};
uint8_t lcdPosition = 0;
char command[48];
uint8_t commandLength = 0;
bool commandOverflow = false;
// Bounded output queue: serial traffic never waits for a full UART buffer.
char tx[256];
uint8_t txHead = 0, txTail = 0;
uint32_t droppedMessages = 0;
bool buttonRaw[4] = {HIGH, HIGH, HIGH, HIGH};
bool buttonStable[4] = {HIGH, HIGH, HIGH, HIGH};
uint32_t buttonChanged[4] = {0, 0, 0, 0};

void queueLine(const char *s) {
  size_t n = strlen(s);
  uint8_t freeBytes = (uint8_t)(txTail - txHead - 1);
  if (n + 1 > freeBytes) { ++droppedMessages; return; }
  while (*s) { tx[txHead++] = *s++; }
  tx[txHead++] = '\n';
}

void queueFlash(const __FlashStringHelper *s) {
  PGM_P p = reinterpret_cast<PGM_P>(s);
  size_t n = strlen_P(p);
  if (n + 1 > (uint8_t)(txTail - txHead - 1)) { ++droppedMessages; return; }
  while (n--) tx[txHead++] = pgm_read_byte(p++);
  tx[txHead++] = '\n';
}

void serviceTx() {
  uint8_t budget = 16;
  while (budget-- && txTail != txHead && Serial.availableForWrite() > 0)
    Serial.write((uint8_t)tx[txTail++]);
}

const char *stateName() {
  return state == RUNNING ? "RUNNING" : state == TRIPPED ? "TRIPPED" : "STARTUP";
}

uint8_t detectFault() {
  uint8_t f = NONE;
  if (voltage10 < vMin * 10U) f |= UV;
  if (voltage10 > vMax * 10U) f |= OV;
  if (current10 > iMax10) f |= OC;
  return f;
}

bool inRecoveryBand() {
  return voltage10 >= (vMin + 10U) * 10U &&
         voltage10 <= (vMax - 10U) * 10U && current10 <= iMax10 - 5U;
}

bool readyToReset(uint32_t now) {
  return haveSample && safeTiming && inRecoveryBand() &&
         (uint32_t)(now - safeSince) >= RECOVERY_MS;
}

void enableLoad() {
  fault = NONE;
  state = RUNNING;
  digitalWrite(FAULT_PIN, LOW);
  digitalWrite(RELAY_PIN, RELAY_ON);
  queueFlash(F("EVENT RUNNING relay=ON"));
}

void tripLoad(uint8_t reason, uint32_t now) {
  // Disconnect BEFORE logging, formatting, or drawing the display.
  digitalWrite(RELAY_PIN, RELAY_OFF);
  digitalWrite(FAULT_PIN, HIGH);
  state = TRIPPED;
  fault = reason;
  tripVoltage10 = voltage10;
  tripCurrent10 = current10;
  tripAtMs = now;
  ++tripCount;
  menuPage = 0;
  char b[100];
  snprintf_P(b, sizeof b, PSTR("EVENT TRIP t=%lu mask=%u V=%u.%u I=%u.%u count=%lu"),
           (unsigned long)now, (unsigned)fault,
           voltage10/10, voltage10%10, current10/10, current10%10,
           (unsigned long)tripCount);
  queueLine(b);
}

void updateProtection(uint32_t now) {
  if (inRecoveryBand()) {
    if (!safeTiming) { safeTiming = true; safeSince = now; }
  } else safeTiming = false;
  uint8_t f = detectFault();
  if (state != TRIPPED && f != NONE) tripLoad(f, now);
  if (state == STARTUP && readyToReset(now)) enableLoad();
}

void sampleInputs() {
  uint32_t nowUs = micros();
  uint32_t gap = (uint32_t)(nowUs - lastSampleUs);
  if (haveSample && gap < SAMPLE_US) return;
  if (haveSample && gap > maxSampleGapUs) maxSampleGapUs = gap;
  lastSampleUs = nowUs;
  voltage10 = ((uint32_t)analogRead(A0) * 3000U + 511U) / 1023U;
  current10 = ((uint32_t)analogRead(A1) * 100U + 511U) / 1023U;
  haveSample = true;
  updateProtection(millis());
}

void requestReset() {
  if (state != TRIPPED) { queueFlash(F("ERR reset requires TRIPPED state")); return; }
  if (!readyToReset(millis())) { queueFlash(F("ERR wait for recovery band for 2 seconds")); return; }
  enableLoad();
}

bool applySetting(uint8_t which, uint16_t value) {
  bool valid = (which == 1 && value >= 150 && value <= 220 && value + 30 <= vMax) ||
               (which == 2 && value >= 230 && value <= 280 && value >= vMin + 30) ||
               (which == 3 && value >= 10 && value <= 90);
  if (!valid) { queueFlash(F("ERR range VMIN=150..220 VMAX=230..280 gap>=30 IMAX=1.0..9.0")); return false; }
  if (which == 1) vMin = value;
  if (which == 2) vMax = value;
  if (which == 3) iMax10 = value;
  safeTiming = false;
  updateProtection(millis()); // new limits also apply while the menu is open
  queueFlash(F("OK setting applied (RAM only)"));
  return true;
}

void printStatus() {
  char b[128];
  snprintf_P(b, sizeof b, PSTR("STATUS t=%lu V=%u.%u I=%u.%u state=%s relay=%s fault=%u ready=%u"),
           (unsigned long)millis(), voltage10/10, voltage10%10,
           current10/10, current10%10, stateName(),
           state == RUNNING ? "ON" : "OFF", (unsigned)fault,
           (unsigned)readyToReset(millis()));
  queueLine(b);
  snprintf_P(b, sizeof b, PSTR("LIMITS VMIN=%u VMAX=%u IMAX=%u.%u gap_us=%lu dropped=%lu"),
           vMin, vMax, iMax10/10, iMax10%10,
           (unsigned long)maxSampleGapUs, (unsigned long)droppedMessages);
  queueLine(b);
}

// Accept decimal digits and at most one fractional digit; reject NaN, suffixes,
// exponents, negatives, overflow, and trailing junk. Result is in tenths.
bool parseTenths(const char *s, uint16_t &out) {
  if (!s || *s < '0' || *s > '9') return false;
  uint16_t whole = 0;
  while (*s >= '0' && *s <= '9') {
    whole = whole * 10U + (*s++ - '0');
    if (whole > 999) return false;
  }
  uint8_t fraction = 0;
  if (*s == '.') {
    ++s;
    if (*s < '0' || *s > '9') return false;
    fraction = *s++ - '0';
  }
  if (*s) return false;
  out = whole * 10U + fraction;
  return true;
}

void handleCommand(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  char *arg = strtok(NULL, " \t");
  char *value = strtok(NULL, " \t");
  char *extra = strtok(NULL, " \t");
  if (!strcmp(cmd, "STATUS") && !arg) { printStatus(); return; }
  if (!strcmp(cmd, "RESET") && !arg) { requestReset(); return; }
  if (!strcmp(cmd, "HELP") && !arg) {
    queueFlash(F("STATUS | RESET | SET VMIN 180 | SET VMAX 250 | SET IMAX 5.0 | STREAM ON/OFF | LAST"));
    queueFlash(F("Fault bits: UV=1 OV=2 OC=4; simultaneous faults add. Defaults return on reboot."));
    return;
  }
  if (!strcmp(cmd, "LAST") && !arg) {
    char b[100];
    snprintf_P(b, sizeof b, PSTR("LAST count=%lu t=%lu V=%u.%u I=%u.%u"),
             (unsigned long)tripCount, (unsigned long)tripAtMs,
             tripVoltage10/10, tripVoltage10%10, tripCurrent10/10, tripCurrent10%10);
    queueLine(b); return;
  }
  if (!strcmp(cmd, "STREAM") && arg && !value && (!strcmp(arg,"ON") || !strcmp(arg,"OFF"))) {
    streamEnabled = !strcmp(arg,"ON"); queueFlash(F("OK stream changed")); return;
  }
  if (!strcmp(cmd, "SET") && arg && value && !extra) {
    uint16_t n;
    if (!parseTenths(value, n)) { queueFlash(F("ERR invalid decimal value")); return; }
    uint8_t which = !strcmp(arg,"VMIN") ? 1 : !strcmp(arg,"VMAX") ? 2 : !strcmp(arg,"IMAX") ? 3 : 0;
    if (!which || (which != 3 && n % 10)) { queueFlash(F("ERR unknown setting or fractional voltage limit")); return; }
    if (applySetting(which, which == 3 ? n : n/10)) menuPage = 0;
    return;
  }
  queueFlash(F("ERR command; type HELP"));
}

void serviceRx() {
  uint8_t budget = 8; // bounded work even under continuous input
  while (budget-- && Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (commandOverflow) queueFlash(F("ERR line too long"));
      else if (commandLength) { command[commandLength] = 0; handleCommand(command); }
      commandLength = 0; commandOverflow = false;
    } else if (commandOverflow) { /* discard until the next line ending */ }
    else if (c == '\b' || c == 127) { if (commandLength) --commandLength; }
    else if (c >= 32 && c <= 126) {
      if (commandLength < sizeof(command)-1) command[commandLength++] = c;
      else commandOverflow = true;
    }
  }
}

void buttonPress(uint8_t key) {
  if (key == 3) { menuPage = 0; requestReset(); return; }
  if (key == 0) {
    uint8_t next = menuPage == 3 ? 0 : menuPage + 1;
    if (menuPage) {
      State before = state;
      if (!applySetting(menuPage, editValue)) return;
      if (before != TRIPPED && state == TRIPPED) return;
    }
    menuPage = next;
    editValue = menuPage == 1 ? vMin : menuPage == 2 ? vMax : iMax10;
  } else if (menuPage) {
    uint16_t lo = menuPage == 1 ? 150 : menuPage == 2 ? 230 : 10;
    uint16_t hi = menuPage == 1 ? 220 : menuPage == 2 ? 280 : 90;
    if (key == 1 && editValue < hi) ++editValue;
    if (key == 2 && editValue > lo) --editValue;
  }
  lastLcdMs = millis() - 200;
}

void serviceButtons() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < 4; ++i) {
    bool raw = digitalRead(BUTTON_PINS[i]);
    if (raw != buttonRaw[i]) { buttonRaw[i] = raw; buttonChanged[i] = now; }
    if (raw != buttonStable[i] && (uint32_t)(now-buttonChanged[i]) >= 25) {
      buttonStable[i] = raw;
      if (raw == LOW) buttonPress(i);
    }
  }
}

void formatLcd() {
  uint32_t now = millis();
  if ((uint32_t)(now-lastLcdMs) < 200) return;
  lastLcdMs = now;
  char a[32], b[32];
  snprintf_P(a, sizeof a, PSTR("V:%3u.%u I:%2u.%u"), voltage10/10, voltage10%10, current10/10, current10%10);
  if (menuPage) {
    const char *name = menuPage == 1 ? "VMIN" : menuPage == 2 ? "VMAX" : "IMAX";
    if (menuPage == 3) snprintf_P(a, sizeof a, PSTR("EDIT %s %u.%uA"), name, editValue/10, editValue%10);
    else snprintf_P(a, sizeof a, PSTR("EDIT %s %uV"), name, editValue);
    snprintf_P(b, sizeof b, PSTR("+/- MENU=apply"));
  } else if (state == RUNNING) snprintf_P(b, sizeof b, PSTR("RUNNING LOAD ON"));
  else if (state == STARTUP) snprintf_P(b, sizeof b, PSTR("STARTUP WAIT 2s"));
  else if (readyToReset(now)) snprintf_P(b, sizeof b, PSTR("SAFE PRESS RESET"));
  else snprintf_P(b, sizeof b, PSTR("TRIP %s%s%s"), fault & UV ? "UV " : "", fault & OV ? "OV " : "", fault & OC ? "OC" : "");
  snprintf_P(desired[0], 17, PSTR("%-16.16s"), a);
  snprintf_P(desired[1], 17, PSTR("%-16.16s"), b);
}

void serviceLcd() {
  // At most one changed character per loop; no full-screen blocking redraw.
  for (uint8_t n = 0; n < 32; ++n) {
    uint8_t pos = lcdPosition++ & 31, row = pos/16, col = pos%16;
    if (shown[row][col] != desired[row][col]) {
      lcd.setCursor(col, row); lcd.write((uint8_t)desired[row][col]);
      shown[row][col] = desired[row][col]; return;
    }
  }
}

void setup() {
  digitalWrite(RELAY_PIN, RELAY_OFF); pinMode(RELAY_PIN, OUTPUT);
  pinMode(FAULT_PIN, OUTPUT); digitalWrite(FAULT_PIN, LOW);
  for (uint8_t i = 0; i < 4; ++i) pinMode(BUTTON_PINS[i], INPUT_PULLUP);
  Serial.begin(115200);
  lcd.begin(16, 2);
  memset(shown, '?', sizeof shown);
  memset(desired, ' ', sizeof desired);
  desired[0][16] = desired[1][16] = 0;
  queueFlash(F("Voltage Protector - EMULATED INPUTS. Type HELP. 115200 baud, newline."));
  sampleInputs();
  lastLcdMs = millis() - 200;
}

void loop() {
  sampleInputs();
  serviceButtons();
  serviceRx();
  sampleInputs();
  uint32_t now = millis();
  if (streamEnabled && (uint32_t)(now-lastStreamMs) >= 1000) {
    lastStreamMs = now; printStatus();
  }
  formatLcd();
  serviceLcd();
  serviceTx();
}
