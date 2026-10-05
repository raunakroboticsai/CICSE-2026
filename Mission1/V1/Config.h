// ======================================================
// Config.h  -  ALL SETTINGS ARE HERE (numbers you tune)
// ======================================================
#ifndef MISSION_CONFIG_H
#define MISSION_CONFIG_H

// ======================================================
//  ONE SWITCH:   SAFE_MODE
//   1 = SAFE      the old, tested wheel movement (fixed motor power, stops by
//                 the encoders). No gyro, no speed control. USE THIS FOR THE COMPETITION
//                 until the advanced parts are tested on the robot.
//   0 = ADVANCED  speed control + MPU6050 gyro + brake/trim. Test first with
//                 E (wheel test), M and R (gyro), Q (speed).
// ======================================================
#define SAFE_MODE 1

#define BUILD_NAME "COMPETITION FINAL 1"

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>

// ---------------- DRIVE MOTORS ----------------
#define M1_PWM 2
#define M1_IN1 31
#define M1_IN2 30

#define M2_PWM 3
#define M2_IN1 32
#define M2_IN2 33

#define M3_PWM 4
#define M3_IN1 35
#define M3_IN2 34

#define M4_PWM 5
#define M4_IN1 36
#define M4_IN2 37

#define SPEED 150
#define SLOW_SPEED 90

#define COUNTS_PER_MM 10.34

#define DIAGONAL_FACTOR 2.20     // ESTIMATED from forward+strafe (check with A: forward and right parts should each be ~212 mm for 300)
#define STRAFE_FACTOR 1.75      // measured: 1.66 gave 285 mm for 300
#define FORWARD_FACTOR 1.43     // measured: 1.36 gave 285 mm for 300
#define BACKWARD_FACTOR 1.43    // measured: 1.26 gave 265 mm for 300

// 1 = on YOUR robot the wheel pattern that the code called "turn right" really turns the robot
//     LEFT (seen in the video test). With 1 the code swaps the two turns, so ROTATE_RIGHT
//     really turns right and ROTATE_LEFT really turns left.
// If the step-3 test (Y key) turns the wrong way, change this 1 to 0 (or 0 to 1).
#define ROTATION_MIRRORED 1

#define TURN_90_WHEEL_MM 380    // measured: 380 gave 90 deg

// 0 = the distance is the plain AVERAGE of all the wheel encoders (the old, tested way).
// 1 = only the MIDDLE wheels count (the highest and the lowest encoder are dropped).
//     Use 1 ONLY if an encoder dies again (E test shows NO COUNTS).
#define ENCODER_TOLERANT  0

// ---------------- MPU6050 GYRO (straight lines + exact turns) ----------------
#define USE_MPU           1      // gyro on (heading hold + exact turns). 0 = off
#define MPU_ADDR          0x68
#define GYRO_AXIS         2      // 0=X 1=Y 2=Z : the gyro axis that points UP
#define GYRO_SIGN        -1      // +1 or -1 : turning RIGHT must make the yaw go UP (test: M key)
#define MPU_CAL_SAMPLES   400    // gyro calibration (robot must stand still)
#define HEADING_KP        0.040  // straight-line correction per degree (fraction of REF speed)
#define HEADING_KD        0.004  // straight-line damping per deg/s
#define HEADING_MAX       0.25   // max correction (fraction of top speed)

// Gyro help for the normal (fixed motor power) driving: while the robot drives, the gyro
// adds a little power to the wheels that turn the robot back to the straight heading.
#define ASSIST_KP_PWM     8.0    // motor power added per degree of heading error
#define ASSIST_KD_PWM     1.5    // ... per degree/second
#define ASSIST_MAX_PWM    40     // never add more than this
#define SYNC_KP_GYRO      0.15   // wheel-progress sync while the gyro holds the heading (weak, so they do not fight)
#define TURN_TOL_DEG      0.4    // a turn is finished when it is this close to the angle
#define TURN_FINE_PWM     110    // motor power of the small final turn pulses
#define TURN_FINE_MS_PER_DEG 30  // pulse length per degree that is still missing (ms)

// ---------------- SPEED CONTROL (battery-proof) ----------------
// Every wheel's real speed is measured by its encoder and the motor power is
// adjusted until the speed is right (PI control). The robot speeds up slowly,
// slows down before the target and creeps the last mm, so it runs at the SAME
// speed and stops at the SAME place on a full or a half-empty battery.
// All speeds are fractions of REF (counts per second the robot makes at SPEED).
// Send Q once (with a half-charged battery) to measure REF.
#define SPEED_CONTROL     (SAFE_MODE ? 0 : 1)   // 1 = closed-loop speed control, 0 = old open-loop
#define REF_CPS_DEFAULT   1500  // used until Q has been done
#define V_MAX_FRAC        0.85  // top speed
#define V_SLOW_FRAC       0.30  // slow moves (search / alignment)
#define V_CREEP_FRAC      0.12  // last mm before the target
#define A_ACC_FRAC        2.5   // speed-up
#define A_DEC_FRAC        2.0   // slow-down before the target
#define CTRL_MS           20    // speed control period
#define SPEED_FILTER      0.4   // speed filter
#define SPEED_GAIN        0.25  // PI gain
#define SYNC_KP           3.0   // keeps all wheels equal (straight line)
#define START_PWM         90    // first motor power

// ---------------- STOPPING (so the distance does not depend on the battery) --------
// After the move the motors are BRAKED (not just switched off) so the robot does
// not coast further on a full battery. If it still went too far, it drives back
// slowly the extra distance (TRIM).
#define BRAKE_MS         (SAFE_MODE ? 0 : 60)   // brake time (ms), 0 = no brake
#define TRIM_ENABLE       (SAFE_MODE ? 0 : 1)   // 1 = drive back if it overshot
#define TRIM_TOL_MM     2.0     // overshoot smaller than this is ignored
#define TRIM_MAX_MS    1500     // max time for the drive back

// ---------------- ARM (PCA9685 0x40) ----------------
#define BASE_SERVO    0
#define BOTTOM_SERVO  1
#define TOP_SERVO     2

#define BASE_MIN    100
#define BASE_MAX    560
#define BOTTOM_MIN  150
#define BOTTOM_MAX  510
#define TOP_MIN     270
#define TOP_MAX     530

// Rest position when power is OFF
#define IDLE_BASE      346
#define IDLE_BOTTOM    500
#define IDLE_TOP       300

// Arm position while the robot drives to the quarantine zone
#define INITIAL_BASE   350
#define INITIAL_BOTTOM 370
#define INITIAL_TOP    465

// SCAN / CARRY pose: camera faces down. The sample is carried in this pose.
#define SCAN_BASE      341
#define SCAN_BOTTOM    328      // tuned by hand with the < > ( ) keys: arm back, height OK
#define SCAN_TOP       351      // tuned by hand with the < > ( ) keys

// PICK pose start values (nozzle on the floor). G teaches the real ones.
// Smaller bottom = lower (150 = lowest). From your first teaching.
#define PICK_BOTTOM    189
#define PICK_TOP       514

// PLACE pose: where the arm goes DOWN to drop the sample (fixed spot,
// straight ahead of the robot, after the mission drive steps).
// Smaller bottom = lower (150 = lowest). Tune with D and + - [ ]
#define PLACE_BASE     SCAN_BASE
#define PLACE_BOTTOM   165
#define PLACE_TOP      445

// HOVER: the arm first goes to a point this much ABOVE the sample / place
// spot, then goes straight down (so it does not drag the sample).
// Bottom + and top - = higher.
#define HOVER_LIFT_BOTTOM   40
#define HOVER_LIFT_TOP     -50

// Arm speeds and pauses (ms)
#define ARM_MOVE_MS     1200
#define ARM_DOWN_MS     1500
#define PICK_DWELL_MS    800   // arm is down, suction ON, wait to grab
#define PLACE_DWELL_MS   800   // suction OFF, wait to let go

// ---------------- SUCTION (pump switch board on a PCA9685 channel) ----------------
// Your pump switch board is driven by a SERVO-LIKE PULSE (tested with PumpTest):
//   1000 us pulse = pump OFF,   2000 us pulse = pump ON
#define SUCTION_SERVO_SWITCH  1      // 1 = servo-pulse switch board (your case)
#define SUCTION_CHANNEL       5      // PCA9685 channel of the pump switch
#define SUCTION_PULSE_ON_US   2000
#define SUCTION_PULSE_OFF_US  1000

// Other ways (only if SUCTION_SERVO_SWITCH is 0):
//   SUCTION_USE_MEGA_PIN 1 = relay / MOSFET signal wire on Mega pin SUCTION_PIN
#define SUCTION_USE_MEGA_PIN  0
#define SUCTION_PIN           7
#define SUCTION_INVERT        0      // 1 = swap ON and OFF

// ---------------- SLAB SERVO ----------------
// Fill in the numbers, then set USE_SLAB_SERVO to 1.
#define USE_SLAB_SERVO   0
#define SLAB_SERVO       3
#define SLAB_CLOSED      300
#define SLAB_OPEN        450
#define SLAB_WAIT_MS     1000

// ---------------- VISION ----------------
#define SAMPLE_TARGET_ID    3      // "sample" class in your model
#define MIN_SCORE           50     // confidence 0-100
#define CONFIRM_FRAMES      3      // frames in a row needed
#define SCAN_TIMEOUT_MS     3000   // look time when aligning

// ---------------- SCAN SWEEP (find the sample anywhere) ----------------
// The arm turns left/right (base servo) and the camera looks at each stop.
// Stops: centre, then left, right, further left, further right ...
// The arm must search ONLY in the quarantine zone, not towards the lab:
//   press  .  (arm base +10) or  ,  (arm base -10) and watch which way the arm goes.
//   The side where the arm goes TOWARDS THE LAB gets a SMALL range (like 20), the quarantine side a big one.
#define SWEEP_PLUS_RANGE  120   // how far the sweep goes in the + direction of the arm base (key  . )
#define SWEEP_MINUS_RANGE 120   // how far the sweep goes in the - direction of the arm base (key  , )
#define SWEEP_STEP         60   // base units between stops (camera view width)
#define SWEEP_MOVE_MS     500   // time to turn between stops
#define SWEEP_SETTLE_MS   200   // wait for the camera to settle
#define SWEEP_LOOK_MS     900   // look time at each stop

// Not found in the sweep? The robot drives and sweeps again, so the whole
// box in front is covered:  row 0 (here) -> FORWARD row -> BACK row.
// Set a distance to 0 to skip that row.
// Extra scan rows made by the ARM (the camera looks FURTHER / CLOSER in front of the robot).
// 0 = off. Find the pose with the keys  < > ( )  (the camera must see a sample from it) and write
// the two numbers here. SCAN_FAR_MM / SCAN_NEAR_MM = how far (mm) that row looks beyond the normal view.
#define SCAN_FAR_BOTTOM     0
#define SCAN_FAR_TOP        0
#define SCAN_FAR_MM        80
#define SCAN_NEAR_BOTTOM    0
#define SCAN_NEAR_TOP       0
#define SCAN_NEAR_MM       60

#define SEARCH_FWD_MM       0
#define SEARCH_BACK_MM      0

// ---------------- AUTO-ALIGN (camera -> arm turn + robot forward/back) ----------------
// The robot looks, then turns the arm and drives forward/back, round after
// round, until the sample is at the taught target pixel (= right under the
// nozzle). The error shrinks every round. Then the arm goes straight down.
#define ALIGN_TOL_PX      5     // done when the error is this small (pixels)
#define ALIGN_MAX_TRIES   8     // max correction rounds
#define ALIGN_GAIN        0.8   // 1.0 = full correction each round, lower = smoother
#define ALIGN_MAX_MM      120   // never go further forward/back than this
#define ALIGN_USE_WHEELS_RADIAL 1   // 1 = the robot may creep forward/back (max ALIGN_MAX_MM) to line up
                                    // 0 = the robot NEVER drives forward/back to pick: only the arm turns
#define ALIGN_MIN_MM      3     // ignore smaller robot moves
#define ALIGN_MIN_BASE    2     // ignore smaller arm turns (servo units)
#define ROW_MIN_MM        3
#define CAL_MOVE_MM       25    // automatic calibration: robot forward distance
#define CAL_BASE_UNITS    12    // automatic calibration: arm turn amount

// ---------------- PICK CHECK (the robot does not leave until it holds the sample) --------
#define PICK_MAX_TRIES    (SAFE_MODE ? 3 : 5)   // tries per sample before it gives up
#define VERIFY_MS         1000  // look time after the pick
#define VERIFY_RADIUS_PX  30    // sample still within this of the target = NOT picked
#define VERIFY_FRAMES     2     // frames in a row that must see it

// ---------------- MISSION PATH  (the 36-step movement table, distances from the video, in mm) ----
// The robot starts in the START zone with its ARM pointing to the LEFT (towards the lab),
// like in the video. Step numbers below are the step numbers of the movement table.
//
//  Rotate step 3: in the video the arm turns from LEFT to DOWN (towards the quarantine / lab),
//  and that is a turn to the LEFT. The table says "Rotate Right". If the robot ends with its arm
//  pointing the wrong way, change P_TURN_RIGHT.
#define P_TURN_RIGHT        0     // 0 = step 3 turns LEFT (really, as in the simulation video), 1 = turns RIGHT

// After the turn the robot ONLY strafes right / left (and the small backward steps 5, 33, 35).
// The steps that are not listed (10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 31, 32) are NOT driven:
// the arm reaches the samples and the lab slots from the same row (tune PLACE_BOTTOM / PLACE_TOP).

// PART A - quarantine zone
#define S01_STRAFE_RIGHT  211     // out of the start zone
#define S02_FORWARD       244
#define S04_STRAFE_RIGHT  379     // (after the 90 degree turn) to the quarantine
#define S04_WALL_EXTRA_MM  75     // step 4 goes this much FURTHER: the robot touches the wall and squares up (tested: 75 is OK)
#define S05_BACKWARD       25
#define S05_EXTRA_MM       35     // step 5 backs up this much MORE before the camera scan
#define S07_STRAFE_LEFT    47     // to sample 1

// PART B - sample 1 to lab slot 1
#define S09_STRAFE_LEFT   195
#define S13_STRAFE_RIGHT  236

// PART C - sample 2 to lab slot 2
#define S17_STRAFE_LEFT   310
#define S21_STRAFE_RIGHT  310

// PART D - sample 3 to lab slot 3
#define S25_STRAFE_LEFT   410
#define S29_STRAFE_RIGHT  414

// Extra reach at the lab: the robot goes this many mm FORWARD, drops the sample, and comes BACK the same
// distance, before the strafe. 0 = no extra move.
#define P1_PLACE_FWD_MM     0     // sample 1
#define P2_PLACE_FWD_MM     0     // sample 2
#define P3_PLACE_FWD_MM   100     // sample 3

// PART E - beam (slab servo) deployment and finish
#define S33_BACKWARD       16
#define S35_BACKWARD       30

// ---------------- GUIDED SETUP (G): ONE spot ----------------
#define GUIDE_STEP        3     // bottom / top key step (servo units)
#define GUIDE_BASE_STEP   2     // base key step

// arm may never turn outside this safe zone
#define BASE_SAFE_MIN  (SCAN_BASE - SWEEP_MINUS_RANGE - 40)
#define BASE_SAFE_MAX  (SCAN_BASE + SWEEP_PLUS_RANGE + 40)

// ---------------- I2C ----------------
#define I2C_SDA_PIN       20
#define I2C_SCL_PIN       21
#define PCA9685_ADDR      0x40

// ---------------- START ----------------
#define START_BUTTON_PIN 22     // START button between pin 22 and GND
#define STOP_BUTTON_PIN  29     // STOP button between pin 29 and GND
#define AUTO_START 0            // 1 = start by itself after the delay
#define AUTO_START_DELAY_MS  5000

// ======================================================
// TYPES
// ======================================================
enum MoveType {
  DIAGONAL_FRONT_RIGHT,
  ROTATE_RIGHT,
  STRAFE_RIGHT,
  BACKWARD,
  STRAFE_LEFT,
  FORWARD,
  ROTATE_LEFT
};

#define SETTINGS_MAGIC 0xB71A

struct Settings {
  unsigned int magic;
  int codePlaceBottom;  // PLACE_BOTTOM in the code when this was saved
  int codePlaceTop;
  int placeBottom;      // tuned place pose
  int placeTop;
  int pickBottom;       // taught pick pose (nozzle on the floor)
  int pickTop;
  int targetX, targetY; // where the camera sees a sample that is under the nozzle
  float bx, by;         // picture shift (px) per 1 arm-turn unit
  float fx, fy;         // picture shift (px) per 1 mm robot FORWARD
  byte targetSet;       // teach done
  byte calibrated;      // automatic calibration done
};

#define MAX_BOXES 10
struct Box {
  int x, y, w, h, score, target;
};

// ======================================================
// SHARED VARIABLES (each is defined in one tab)
// ======================================================
extern Settings cfg;
extern Adafruit_PWMServoDriver pwm;
extern bool missionAborted;
extern bool anyKeySeen;
extern bool suctionActive;
extern byte stepCounter;
extern float armBase, armBottom, armTop;
extern bool armDriverOk;
extern bool visionOk;
extern Box boxes[MAX_BOXES];
extern float refCps;
extern float yawDeg, yawRate, headingRef, moveYaw0;
extern bool mpuOk;
extern bool moveAllowStall;
extern int boxCount;

// ======================================================
// FUNCTION LIST
// ======================================================
// Mission1.ino
bool mdrive(MoveType t, float mm, float slow);
bool mdriveWall(MoveType t, float mm, float slow);
bool mgate(const __FlashStringHelper *name);
bool stepWait();
void turnTest();
bool abortRequested();
bool runMission();
void startMission();
void printHelp();
void showPlacePose();
void preflight();
void changePlaceHeight(int delta);
void changePlaceTop(int delta);
void nudgeTarget(int dx, int dy);
void forgetSettings();
void checkSerial();
void checkStartButton();

// Movement.ino
void encStep(byte m, byte now);
void encodersBegin();
void readEnc(long result[4]);
void resetEncoders();
void motorRun(byte pwmPin, byte in1, byte in2, int speed);
void stopMotors();
void brakeMotors(unsigned int ms);
void settleWait(unsigned long ms);
float moveProgress(MoveType type);
void wheelDirs(MoveType type, int8_t d[4]);
bool moveFinish(MoveType type, float targetCounts);
bool encoderMoveOpen(float mm, float factor, MoveType type, float slowDistance);
bool encoderMoveClosed(float mm, float factor, MoveType type, float slowDistance);
void speedLoad();
void speedTest();
void wheelTest();
void runMovement(MoveType type, int spd);
void runMovementAssist(MoveType type, int spd);
bool encoderMove(float mm, float factor, MoveType type, float slowDistance);
const char *moveName(MoveType type);
float moveFactor(MoveType type);
bool smallMove(MoveType type, float mm);
bool drive(MoveType type, float mm, float slowDistance);

// Arm.ino
void armApply();
bool armMoveTo(int b, int bo, int t, unsigned long durationMs);
bool armToInitial();
bool armToScan();
bool armLiftHere();
bool armOver(float base, float bottom, float top);
bool armDown(float base, float bottom, float top);
void setSuction(bool on);
bool waitMs(unsigned long ms);
bool openSlab();
void slabTest();

// Mpu.ino
bool mpuBegin();
bool mpuCalibrate();
void mpuUpdate();
void mpuTest();

// Vision.ino
bool sampleNear(int tx, int ty, int radius, unsigned long timeoutMs, int &ox, int &oy);
bool gv2Present();
void gv2Write(const char *data, uint16_t len);
uint16_t gv2Available();
uint8_t gv2Read(uint8_t *buf, uint8_t len);
void gv2Flush();
void gv2ParseBoxes(const char *p);
int gv2HandleLine();
int gv2Invoke();
bool lookForSample(int &x, int &y, unsigned long timeoutMs);

// Pick.ino
void settingsDefaults();
void settingsSave();
void settingsLoad();
bool alignReady();
void showSettings();
int buildSweep(int offs[]);
bool findSample(int &x, int &y, float &offF, bool allowRows);
bool undoMove(float offF);
bool alignToSample(int x, int y, float &offF);
bool calibrateCamera();
char waitKey();
void guidedSetup();
void findTest();
bool pickSample(byte n);
bool placeSample(byte n);

#endif