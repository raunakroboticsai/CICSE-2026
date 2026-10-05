// ======================================================
// Pick.ino  -  FIND the sample, ALIGN to it by itself, PICK, PLACE
// (settings saved in EEPROM live here too)
//
// HOW A PICK WORKS  (fully automatic)
//  1. SWEEP: arm turns left/right, camera looks at each stop.
//     Not found? robot drives forward / back and sweeps again.
//  2. ALIGN: camera looks, the arm turns and the robot drives
//     forward/back, round after round, until the sample is at the
//     taught target pixel (= right under the nozzle). Error shrinks
//     every round.
//  3. Arm goes over the sample, suction ON, goes straight down,
//     waits, lifts.
//  4. Arm goes straight (carry pose), robot returns to its stop point.
//     Suction stays ON until PLACE.
//
// G teaches ONE spot (where the nozzle is in the picture) and then the
// robot calibrates itself (turns the arm, drives 25 mm).
// ======================================================
#include "Config.h"

Settings cfg;

// ------------------------------------------------------
// SAVED SETTINGS (EEPROM)
// ------------------------------------------------------
void settingsDefaults() {
  cfg.magic = SETTINGS_MAGIC;
  cfg.codePlaceBottom = PLACE_BOTTOM;
  cfg.codePlaceTop = PLACE_TOP;
  cfg.placeBottom = PLACE_BOTTOM;
  cfg.placeTop = PLACE_TOP;
  cfg.pickBottom = PICK_BOTTOM;
  cfg.pickTop = PICK_TOP;
  cfg.targetX = 0;
  cfg.targetY = 0;
  cfg.bx = cfg.by = cfg.fx = cfg.fy = 0;
  cfg.targetSet = 0;
  cfg.calibrated = 0;
}

void settingsSave() {
  cfg.magic = SETTINGS_MAGIC;
  cfg.codePlaceBottom = PLACE_BOTTOM;
  cfg.codePlaceTop = PLACE_TOP;
  EEPROM.put(0, cfg);
}

void settingsLoad() {
  EEPROM.get(0, cfg);

  if (cfg.magic != SETTINGS_MAGIC) {
    settingsDefaults();
    return;
  }

  // PLACE_BOTTOM / PLACE_TOP edited in the code: the code wins
  if (cfg.codePlaceBottom != PLACE_BOTTOM) {
    cfg.placeBottom = PLACE_BOTTOM;
    cfg.codePlaceBottom = PLACE_BOTTOM;
  }
  if (cfg.codePlaceTop != PLACE_TOP) {
    cfg.placeTop = PLACE_TOP;
    cfg.codePlaceTop = PLACE_TOP;
  }

  cfg.placeBottom = constrain(cfg.placeBottom, BOTTOM_MIN, BOTTOM_MAX);
  cfg.placeTop = constrain(cfg.placeTop, TOP_MIN, TOP_MAX);
  cfg.pickBottom = constrain(cfg.pickBottom, BOTTOM_MIN, BOTTOM_MAX);
  cfg.pickTop = constrain(cfg.pickTop, TOP_MIN, TOP_MAX);
}

bool alignReady() {
  return cfg.targetSet && cfg.calibrated;
}

void showSettings() {
  Serial.println(F("----- saved settings -----"));
  Serial.print(F("Pick pose:  bottom="));
  Serial.print(cfg.pickBottom);
  Serial.print(F(" top="));
  Serial.println(cfg.pickTop);
  Serial.print(F("Place pose: bottom="));
  Serial.print(cfg.placeBottom);
  Serial.print(F(" top="));
  Serial.println(cfg.placeTop);

  Serial.print(F("Target (G): "));
  if (cfg.targetSet) {
    Serial.print(F("x="));
    Serial.print(cfg.targetX);
    Serial.print(F(" y="));
    Serial.println(cfg.targetY);
  } else {
    Serial.println(F("NOT DONE"));
  }

  Serial.print(F("Auto calibration: "));
  if (cfg.calibrated) {
    Serial.print(F("arm turn "));
    Serial.print(cfg.bx);
    Serial.print(F(","));
    Serial.print(cfg.by);
    Serial.print(F(" px/unit   forward "));
    Serial.print(cfg.fx);
    Serial.print(F(","));
    Serial.print(cfg.fy);
    Serial.println(F(" px/mm"));
  } else {
    Serial.println(F("NOT DONE"));
  }

  Serial.println(alignReady() ? F("Pick mode: AUTO (find + align + pick)")
                              : F("Pick mode: NOT READY. Send G first!"));
  Serial.println(F("--------------------------"));
}

// ------------------------------------------------------
// SWEEP: find the sample anywhere in front of the robot
// ------------------------------------------------------
// The sweep stops (arm base offsets from SCAN_BASE): the centre first, then step by step to each
// side, nearest first. Each side has its own range (SWEEP_PLUS_RANGE / SWEEP_MINUS_RANGE), so the
// arm can be kept away from the lab side.
int buildSweep(int offs[]) {
  int n = 0;
  offs[n++] = 0;

  for (int k = 1; n < 14; k++) {
    int m = k * SWEEP_STEP;
    bool any = false;
    if (m <= SWEEP_MINUS_RANGE) { offs[n++] = -m; any = true; }
    if (m <= SWEEP_PLUS_RANGE)  { offs[n++] =  m; any = true; }
    if (!any) break;
  }

  return n;
}

// One scan row of the ARM: a scan pose that looks further in front of the robot (far),
// closer (near), or the normal one (main).
struct ScanRow {
  int bottom;
  int top;
  float mm;        // 0 for the normal row
};

// Looks for the sample: the arm sweeps to each allowed stop on the normal row, then on the far
// and near rows (if they are set in Config.h). The robot itself only moves when it has to.
// On success the arm stays turned where it saw the sample (armBase) in the NORMAL scan pose.
// offF = how far the robot is from its stop point now (mm, forward +).
bool findSample(int &x, int &y, float &offF, bool allowRows) {
  float rowOff[3];
  int rows = 0;

  rowOff[rows++] = 0;
  if (allowRows && SEARCH_FWD_MM > 0)  rowOff[rows++] = SEARCH_FWD_MM;
  if (allowRows && SEARCH_BACK_MM > 0) rowOff[rows++] = -SEARCH_BACK_MM;

  ScanRow arm[3];
  int armRows = 0;
  arm[armRows++] = {SCAN_BOTTOM, SCAN_TOP, 0};
#if SCAN_FAR_BOTTOM > 0
  arm[armRows++] = {SCAN_FAR_BOTTOM, SCAN_FAR_TOP, (float)SCAN_FAR_MM};
#endif
#if SCAN_NEAR_BOTTOM > 0
  arm[armRows++] = {SCAN_NEAR_BOTTOM, SCAN_NEAR_TOP, -(float)SCAN_NEAR_MM};
#endif

  for (int r = 0; r < rows; r++) {
    float delta = rowOff[r] - offF;

    if (fabs(delta) >= ROW_MIN_MM) {
      Serial.println(F("Not found here - moving to search another row"));
      if (!smallMove(delta > 0 ? FORWARD : BACKWARD, fabs(delta))) return false;
      offF = rowOff[r];
    }

    int offs[16];
    int stops = buildSweep(offs);

    for (int a = 0; a < armRows; a++) {
      if (a > 0) Serial.println(a == 1 && SCAN_FAR_BOTTOM > 0 ? F("Arm scan row: FAR") : F("Arm scan row: NEAR"));

      for (int i = 0; i < stops; i++) {
        int b = SCAN_BASE + offs[i];

        Serial.print(F("Sweep: arm base "));
        Serial.println(b);

        if (!armMoveTo(b, arm[a].bottom, arm[a].top, (i == 0) ? ARM_MOVE_MS : SWEEP_MOVE_MS)) return false;
        if (!waitMs(SWEEP_SETTLE_MS)) return false;

        if (!lookForSample(x, y, SWEEP_LOOK_MS)) {
          if (missionAborted) return false;
          continue;
        }

        if (a == 0) return true;                 // seen from the normal scan pose: done

        // seen from the far / near row: bring the arm to the normal scan pose at this angle
        Serial.println(F("Seen on an arm row - checking from the normal scan pose"));
        if (!armMoveTo((int)armBase, SCAN_BOTTOM, SCAN_TOP, 500)) return false;
        if (!waitMs(300)) return false;
        if (lookForSample(x, y, SWEEP_LOOK_MS)) return true;
        if (missionAborted) return false;

        // not in the normal view: the robot creeps by the distance of that row, and looks again
        float shift = arm[a].mm;
        Serial.println(F("Not in the normal view - creeping to it"));
        if (!smallMove(shift > 0 ? FORWARD : BACKWARD, fabs(shift))) return false;
        offF += shift;
        if (!waitMs(300)) return false;
        if (lookForSample(x, y, SCAN_TIMEOUT_MS)) return true;
        if (missionAborted) return false;

        // lost: go back and keep searching
        if (!smallMove(shift > 0 ? BACKWARD : FORWARD, fabs(shift))) return false;
        offF -= shift;
      }
    }
  }

  Serial.println(F("Sweep: sample NOT found"));
  return false;
}

// Drive back to the stop point so the next mission moves stay correct
bool undoMove(float offF) {
  if (fabs(offF) >= ROW_MIN_MM) {
    if (!smallMove(offF > 0 ? BACKWARD : FORWARD, fabs(offF))) return false;
  }
  return true;
}

// ------------------------------------------------------
// ALIGN: turn the arm + drive forward/back until the sample is at the
// taught target pixel. Round after round the error gets smaller.
// ------------------------------------------------------
// offF = robot forward offset so far (mm). Updated here.
// Returns false only if stopped (X) or a move failed.
bool alignToSample(int x, int y, float &offF) {
  float det = cfg.bx * cfg.fy - cfg.fx * cfg.by;

  for (int attempt = 0; attempt < ALIGN_MAX_TRIES; attempt++) {
    float dx = cfg.targetX - x;
    float dy = cfg.targetY - y;

    Serial.print(F("ALIGN round "));
    Serial.print(attempt + 1);
    Serial.print(F(": error dx="));
    Serial.print(dx);
    Serial.print(F(" dy="));
    Serial.println(dy);

    if (fabs(dx) <= ALIGN_TOL_PX && fabs(dy) <= ALIGN_TOL_PX) {
      Serial.println(F("ALIGNED: sample is under the nozzle"));
      return true;
    }

    // s = arm turn (servo units), f = robot forward (mm)
#if ALIGN_USE_WHEELS_RADIAL
    float s = ALIGN_GAIN * (dx * cfg.fy - cfg.fx * dy) / det;
    float f = ALIGN_GAIN * (cfg.bx * dy - dx * cfg.by) / det;
#else
    // the robot does not drive forward/back: only the arm turns, as close as it can get
    float s = ALIGN_GAIN * (cfg.bx * dx + cfg.by * dy) / (cfg.bx * cfg.bx + cfg.by * cfg.by);
    float f = 0;
#endif

    float newBase = constrain(armBase + s, BASE_SAFE_MIN, BASE_SAFE_MAX);
    s = newBase - armBase;
    f = constrain(offF + f, -ALIGN_MAX_MM, ALIGN_MAX_MM) - offF;

    bool moved = false;

    if (fabs(s) >= ALIGN_MIN_BASE) {
      if (!armMoveTo((int)(newBase + 0.5), SCAN_BOTTOM, SCAN_TOP, 300)) return false;
      moved = true;
    }

    if (fabs(f) >= ALIGN_MIN_MM) {
      if (!smallMove(f > 0 ? FORWARD : BACKWARD, fabs(f))) return false;
      offF += f;
      moved = true;
    }

    if (!moved) {
      Serial.println(F("ALIGN: as close as it can get"));
      return true;
    }

    if (!waitMs(300)) return false;

    if (!lookForSample(x, y, SCAN_TIMEOUT_MS)) {
      if (missionAborted) return false;
      Serial.println(F("ALIGN: sample lost after moving, picking here"));
      return true;
    }
  }

  Serial.println(F("ALIGN: rounds used up, picking here"));
  return true;
}

// ------------------------------------------------------
// AUTOMATIC CALIBRATION: the robot learns by itself how the picture moves
// when the arm turns and when the robot drives forward.
// The sample must be in the picture. WHEELS MOVE 25 mm forward and back.
// ------------------------------------------------------
bool calibrateCamera() {
  int x0, y0, x1, y1, x2, y2;

  Serial.println(F("CALIBRATE: the arm turns a little, the robot goes forward and back."));

  float b0 = armBase;
  if (!armLiftHere()) return false;
  if (!waitMs(400)) return false;
  if (!lookForSample(x0, y0, SCAN_TIMEOUT_MS)) {
    Serial.println(F("CALIBRATE failed: sample not seen."));
    return false;
  }

  // 1) turn the arm
  if (!armMoveTo((int)(b0 + CAL_BASE_UNITS), SCAN_BOTTOM, SCAN_TOP, 400)) return false;
  if (!waitMs(400)) return false;
  bool okB = lookForSample(x1, y1, SCAN_TIMEOUT_MS);
  if (!armMoveTo((int)b0, SCAN_BOTTOM, SCAN_TOP, 400)) return false;
  if (!waitMs(300)) return false;

  // 2) drive forward
  if (!smallMove(FORWARD, CAL_MOVE_MM)) return false;
  if (!waitMs(400)) return false;
  bool okF = lookForSample(x2, y2, SCAN_TIMEOUT_MS);
  if (!smallMove(BACKWARD, CAL_MOVE_MM)) return false;

  if (!okB || !okF) {
    Serial.println(F("CALIBRATE failed: sample left the picture. Put it nearer the middle and try again (C)."));
    return false;
  }

  float bx = (float)(x1 - x0) / CAL_BASE_UNITS;
  float by = (float)(y1 - y0) / CAL_BASE_UNITS;
  float fx = (float)(x2 - x0) / CAL_MOVE_MM;
  float fy = (float)(y2 - y0) / CAL_MOVE_MM;
  float det = bx * fy - fx * by;

  if (det < 0.02 && det > -0.02) {
    Serial.println(F("CALIBRATE failed: the picture did not move enough. Did the arm / wheels move?"));
    return false;
  }

  cfg.bx = bx; cfg.by = by;
  cfg.fx = fx; cfg.fy = fy;
  cfg.calibrated = 1;
  settingsSave();

  Serial.println(F("CALIBRATION saved."));
  showSettings();
  return true;
}

// ------------------------------------------------------
// GUIDED SETUP (G): show ONE spot, the robot does the rest
// ------------------------------------------------------
// Wait for one key (letters come back as capitals)
char waitKey() {
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r' || c == ' ') continue;
      if (c >= 'a' && c <= 'z') c -= 32;
      return c;
    }
  }
}

void guidedSetup() {
  float mBase = SCAN_BASE;
  float mBottom = cfg.pickBottom;
  float mTop = cfg.pickTop;

  Serial.println();
  Serial.println(F("=== GUIDED SETUP (only ONE spot) ===   (X = cancel any time)"));
  Serial.println(F("STEP 1: put ONE sample on the floor, straight in front of the arm,"));
  Serial.println(F("        not too far. Send Y."));

  // look first: the camera must see it
  while (true) {
    char c = waitKey();
    if (c == 'X') { Serial.println(F("Cancelled.")); return; }
    if (c != 'Y') continue;

    if (!armToScan()) return;
    if (!waitMs(400)) return;

    int x0, y0;
    if (lookForSample(x0, y0, SCAN_TIMEOUT_MS)) {
      Serial.print(F("   OK, the camera sees it at x="));
      Serial.print(x0);
      Serial.print(F(" y="));
      Serial.println(y0);
      break;
    }
    Serial.println(F("   The camera does NOT see the sample. Move it nearer the middle"));
    Serial.println(F("   in front of the arm and send Y again."));
  }

  if (!armOver(mBase, mBottom, mTop)) return;
  if (!armDown(mBase, mBottom, mTop)) return;

  Serial.println(F("STEP 2: the arm is DOWN. Make the nozzle touch the sample EXACTLY:"));
  Serial.println(F("   , . = turn left / right     - + = lower arm down / up"));
  Serial.println(F("   [ ] = upper arm             (you may also slide the sample by hand)"));
  Serial.println(F("   Send Y when the nozzle is exactly on the sample."));

  while (true) {
    char c = waitKey();
    if (c == 'Y') break;
    if (c == 'X') { Serial.println(F("Cancelled.")); return; }

    if (c == ',') mBase   -= GUIDE_BASE_STEP;
    else if (c == '.') mBase += GUIDE_BASE_STEP;
    else if (c == '-') mBottom -= GUIDE_STEP;
    else if (c == '+') mBottom += GUIDE_STEP;
    else if (c == '[') mTop    -= GUIDE_STEP;
    else if (c == ']') mTop    += GUIDE_STEP;
    else continue;

    mBase   = constrain(mBase,   BASE_MIN,   BASE_MAX);
    mBottom = constrain(mBottom, BOTTOM_MIN, BOTTOM_MAX);
    mTop    = constrain(mTop,    TOP_MIN,    TOP_MAX);

    if (!armMoveTo((int)mBase, (int)mBottom, (int)mTop, 250)) return;

    Serial.print(F("   base="));
    Serial.print((int)mBase);
    Serial.print(F(" bottom="));
    Serial.print((int)mBottom);
    Serial.print(F(" top="));
    Serial.println((int)mTop);
  }

  // lift straight up (same angle) and let the camera see where the sample is
  Serial.println(F("STEP 3: taking the picture. Do not touch the sample..."));
  if (!armLiftHere()) return;
  if (!waitMs(500)) return;

  int x, y;
  if (!lookForSample(x, y, SCAN_TIMEOUT_MS)) {
    Serial.println(F("The camera cannot see the sample from here. Move it nearer the"));
    Serial.println(F("middle of the picture and send G again."));
    return;
  }

  cfg.pickBottom = (int)mBottom;
  cfg.pickTop = (int)mTop;
  cfg.targetX = x;
  cfg.targetY = y;
  cfg.targetSet = 1;
  cfg.calibrated = 0;
  settingsSave();

  Serial.print(F("TARGET saved: x="));
  Serial.print(x);
  Serial.print(F(" y="));
  Serial.println(y);

  Serial.println(F("STEP 4: now the ROBOT calibrates itself. The arm will turn a little and"));
  Serial.println(F("        the robot will drive 25 mm forward and back."));
  Serial.println(F("        Do not touch the sample, keep the area clear. Send Y."));

  char c = waitKey();
  if (c != 'Y') { Serial.println(F("Cancelled. Send C later to calibrate.")); return; }

  if (calibrateCamera()) {
    Serial.println(F("SETUP DONE. Put a sample anywhere in front and send K to test."));
  } else {
    Serial.println(F("Calibration not done. Put the sample near the middle and send C."));
  }
}

// ------------------------------------------------------
// TEST: F = sweep only
// ------------------------------------------------------
void findTest() {
  int x, y;
  float offF = 0;

  if (!armToScan()) return;
  if (!waitMs(300)) return;

  if (!findSample(x, y, offF, false)) return;

  Serial.print(F("FOUND at arm base="));
  Serial.print((int)armBase);
  Serial.print(F("  x="));
  Serial.print(x);
  Serial.print(F(" y="));
  Serial.println(y);

  if (cfg.targetSet) {
    Serial.print(F("Distance to target: dx="));
    Serial.print(cfg.targetX - x);
    Serial.print(F(" dy="));
    Serial.println(cfg.targetY - y);
  }

  Serial.println(F("Arm stays turned. H = back to scan pose."));
}

// ------------------------------------------------------
// MISSION ACTIONS
// ------------------------------------------------------
// Find the sample, align to it by itself, pick, CHECK that it is really picked,
// come back. The robot does not leave until the sample is picked (or after
// PICK_MAX_TRIES failed tries). Ends in the carry pose, sample held by suction.
bool pickSample(byte n) {
  Serial.println();
  Serial.print(F(">>> SAMPLE "));
  Serial.print(n);
  Serial.println(F(": FIND + ALIGN + PICK"));

  if (!alignReady()) {
    Serial.println(F("!!! NOT READY. Send G first. Pick skipped."));
    return true;
  }

  int x = 0, y = 0;
  float offF = 0;

  if (!armToScan()) return false;
  if (!waitMs(300)) return false;       // let the camera settle

  bool seen = findSample(x, y, offF, true);
  if (missionAborted) return false;

  if (!seen) {
    Serial.println(F("SAMPLE NOT FOUND - pick skipped"));
    if (!undoMove(offF)) return false;
    return armToScan();
  }

  bool picked = false;

  for (int attempt = 1; attempt <= PICK_MAX_TRIES && !picked; attempt++) {
    if (attempt > 1) {
      Serial.print(F("PICK TRY "));
      Serial.println(attempt);
    }

    if (!alignToSample(x, y, offF)) return false;

    float base = armBase;                       // angle of the sample
    if (!armOver(base, cfg.pickBottom, cfg.pickTop)) return false;
    setSuction(true);                           // ON only now, stays ON until PLACE
    if (!armDown(base, cfg.pickBottom, cfg.pickTop)) return false;
    if (!waitMs(PICK_DWELL_MS)) return false;   // let it grab
    if (!armOver(base, cfg.pickBottom, cfg.pickTop)) return false;
    if (!armLiftHere()) return false;           // up to the camera height, same angle
    if (!waitMs(300)) return false;

    // CHECK: a sample still lying where it was = the pick failed
    int vx, vy;
    if (sampleNear(cfg.targetX, cfg.targetY, VERIFY_RADIUS_PX, VERIFY_MS, vx, vy)) {
      if (missionAborted) return false;
      Serial.println(F("CHECK: sample is STILL on the floor - pick FAILED, trying again"));
      setSuction(false);
      if (!waitMs(300)) return false;
      x = vx;
      y = vy;
    } else {
      if (missionAborted) return false;
      Serial.println(F("CHECK: sample is gone from the floor - PICK OK"));
      picked = true;
    }
  }

  if (!picked) {
    Serial.println(F("!!! PICK FAILED after all tries - going on"));
    setSuction(false);
  }

  if (!armToScan()) return false;             // carry pose (straight)
  if (!undoMove(offF)) return false;          // robot back to the stop point

  if (picked) {
    Serial.print(F("PICKED sample "));
    Serial.println(n);
  }
  return true;
}

// Put the sample down. Suction goes OFF here. Ends in the scan pose.
bool placeSample(byte n) {
  Serial.println();
  Serial.print(F(">>> SAMPLE "));
  Serial.print(n);
  Serial.println(F(": PLACE"));

  if (!armOver(PLACE_BASE, cfg.placeBottom, cfg.placeTop)) return false;
  if (!armDown(PLACE_BASE, cfg.placeBottom, cfg.placeTop)) return false;
  setSuction(false);
  if (!waitMs(PLACE_DWELL_MS)) return false;
  if (!armOver(PLACE_BASE, cfg.placeBottom, cfg.placeTop)) return false;
  if (!armToScan()) return false;

  Serial.print(F("PLACED sample "));
  Serial.println(n);
  return true;
}