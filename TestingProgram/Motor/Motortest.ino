// =====================================================
// MECANUM MOTOR TEST
// Arduino Mega 2560 + 2x TB6612FNG
//
// U2 = M1, M2
// U3 = M3, M4
//
// STBY = Hard-wired to +5V
// =====================================================


// ---------- U2 ----------
#define M1_PWM  2
#define M1_IN1  31
#define M1_IN2  30

#define M2_PWM  3
#define M2_IN1  32
#define M2_IN2  33


// ---------- U3 ----------
#define M3_PWM  4
#define M3_IN1  35
#define M3_IN2  34

#define M4_PWM  5
#define M4_IN1  36
#define M4_IN2  37


#define SPEED 150
#define MOVE_TIME 2500


// =====================================================
// MOTOR 1
// =====================================================

void motor1(int speed) {

  if (speed > 0) {
    digitalWrite(M1_IN1, HIGH);
    digitalWrite(M1_IN2, LOW);
  }
  else if (speed < 0) {
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, HIGH);
  }
  else {
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
  }

  analogWrite(M1_PWM, abs(speed));
}


// =====================================================
// MOTOR 2
// =====================================================

void motor2(int speed) {

  if (speed > 0) {
    digitalWrite(M2_IN1, HIGH);
    digitalWrite(M2_IN2, LOW);
  }
  else if (speed < 0) {
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, HIGH);
  }
  else {
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, LOW);
  }

  analogWrite(M2_PWM, abs(speed));
}


// =====================================================
// MOTOR 3
// =====================================================

void motor3(int speed) {

  if (speed > 0) {
    digitalWrite(M3_IN1, HIGH);
    digitalWrite(M3_IN2, LOW);
  }
  else if (speed < 0) {
    digitalWrite(M3_IN1, LOW);
    digitalWrite(M3_IN2, HIGH);
  }
  else {
    digitalWrite(M3_IN1, LOW);
    digitalWrite(M3_IN2, LOW);
  }

  analogWrite(M3_PWM, abs(speed));
}


// =====================================================
// MOTOR 4
// =====================================================

void motor4(int speed) {

  if (speed > 0) {
    digitalWrite(M4_IN1, HIGH);
    digitalWrite(M4_IN2, LOW);
  }
  else if (speed < 0) {
    digitalWrite(M4_IN1, LOW);
    digitalWrite(M4_IN2, HIGH);
  }
  else {
    digitalWrite(M4_IN1, LOW);
    digitalWrite(M4_IN2, LOW);
  }

  analogWrite(M4_PWM, abs(speed));
}


// =====================================================
// STOP
// =====================================================

void stopMotors() {

  motor1(0);
  motor2(0);
  motor3(0);
  motor4(0);
}


// =====================================================
// FORWARD
// =====================================================

void forward() {

  motor1(SPEED);
  motor2(SPEED);
  motor3(SPEED);
  motor4(SPEED);
}


// =====================================================
// BACKWARD
// =====================================================

void backward() {

  motor1(-SPEED);
  motor2(-SPEED);
  motor3(-SPEED);
  motor4(-SPEED);
}


// =====================================================
// STRAFE RIGHT
// =====================================================

void strafeRight() {

  motor1(SPEED);
  motor2(-SPEED);
  motor3(-SPEED);
  motor4(SPEED);
}


// =====================================================
// STRAFE LEFT
// =====================================================

void strafeLeft() {

  motor1(-SPEED);
  motor2(SPEED);
  motor3(SPEED);
  motor4(-SPEED);
}


// =====================================================
// ROTATE RIGHT
//
// M1 = BACKWARD
// M2 = BACKWARD
// M3 = FORWARD
// M4 = FORWARD
// =====================================================

void rotateRight() {

  motor1(-SPEED);
  motor2(-SPEED);
  motor3(SPEED);
  motor4(SPEED);
}


// =====================================================
// ROTATE LEFT
//
// M1 = FORWARD
// M2 = FORWARD
// M3 = BACKWARD
// M4 = BACKWARD
// =====================================================

void rotateLeft() {

  motor1(SPEED);
  motor2(SPEED);
  motor3(-SPEED);
  motor4(-SPEED);
}


// =====================================================
// DIAGONAL FRONT-RIGHT
//
// M1 = FORWARD
// M4 = FORWARD
// M2 = STOP
// M3 = STOP
// =====================================================

void diagonalFrontRight() {

  motor1(SPEED);
  motor2(0);
  motor3(0);
  motor4(SPEED);
}


// =====================================================
// DIAGONAL FRONT-LEFT
//
// M2 = FORWARD
// M3 = FORWARD
// M1 = STOP
// M4 = STOP
// =====================================================

void diagonalFrontLeft() {

  motor1(0);
  motor2(SPEED);
  motor3(SPEED);
  motor4(0);
}


// =====================================================
// DIAGONAL BACK-RIGHT
//
// M2 = BACKWARD
// M3 = BACKWARD
// M1 = STOP
// M4 = STOP
// =====================================================

void diagonalBackRight() {

  motor1(0);
  motor2(-SPEED);
  motor3(-SPEED);
  motor4(0);
}


// =====================================================
// DIAGONAL BACK-LEFT
//
// M1 = BACKWARD
// M4 = BACKWARD
// M2 = STOP
// M3 = STOP
// =====================================================

void diagonalBackLeft() {

  motor1(-SPEED);
  motor2(0);
  motor3(0);
  motor4(-SPEED);
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  // M1
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  // M2
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  // M3
  pinMode(M3_PWM, OUTPUT);
  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);

  // M4
  pinMode(M4_PWM, OUTPUT);
  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);

  stopMotors();

  delay(1000);
}


// =====================================================
// MAIN TEST SEQUENCE
// =====================================================

void loop() {

  // -----------------------------------------------
  // 1. FORWARD
  // -----------------------------------------------

  forward();
  delay(MOVE_TIME);

  stopMotors();
  delay(500);


  // -----------------------------------------------
  // 2. BACKWARD
  // -----------------------------------------------

  backward();
  delay(MOVE_TIME);

  stopMotors();
  delay(500);


  // -----------------------------------------------
  // 3. STRAFE RIGHT
  // -----------------------------------------------

  strafeRight();
  delay(MOVE_TIME);

  stopMotors();
  delay(500);


  // -----------------------------------------------
  // 4. STRAFE LEFT
  // -----------------------------------------------

  strafeLeft();
  delay(MOVE_TIME);

  stopMotors();
  delay(500);


  // -----------------------------------------------
  // 5. ROTATE RIGHT
  // -----------------------------------------------

  rotateRight();
  delay(MOVE_TIME);

  stopMotors();
  delay(500);


  // -----------------------------------------------
  // 6. ROTATE LEFT
  // -----------------------------------------------

  rotateLeft();
  delay(MOVE_TIME);

  stopMotors();
  delay(500);


  // -----------------------------------------------
  // 7. DIAGONAL FRONT-RIGHT
  // -----------------------------------------------

  diagonalFrontRight();
  delay(MOVE_TIME);

  stopMotors();
  delay(500);


  // -----------------------------------------------
  // 8. DIAGONAL FRONT-LEFT
  // -----------------------------------------------

  diagonalFrontLeft();
  delay(MOVE_TIME);

  stopMotors();
  delay(500);


  // -----------------------------------------------
  // 9. DIAGONAL BACK-RIGHT
  // -----------------------------------------------

  diagonalBackRight();
  delay(MOVE_TIME);

  stopMotors();
  delay(500);


  // -----------------------------------------------
  // 10. DIAGONAL BACK-LEFT
  // -----------------------------------------------

  diagonalBackLeft();
  delay(MOVE_TIME);

  stopMotors();

  // Wait before repeating
  delay(2000);
}
