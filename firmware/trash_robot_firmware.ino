/*****************************************************
 Unified Robot Firmware v2c (No Gyroscope)
 ------------------------------------------
 - Independent FWD/BWD wheel compensation
   FWD_LEFT_ADJ / FWD_RIGHT_ADJ  (forward)
   BWD_LEFT_ADJ / BWD_RIGHT_ADJ  (backward)
 - Legacy SET:LEFT_ADJ / SET:RIGHT_ADJ sets both groups
 - PID stepping (FRONTPID/WALIGN) uses corresponding FWD/BWD comp
 - PID + Photo Sync fully supported
 - SEQ:AUTO, SEQ:METAL, SEQ:PLASTIC
 - All forward/backward via FRONTPID (ultrasonic stepping)
 - All rotation is timed (no gyroscope)
 - Phase 2: wall-align after 180-degree rotation
*****************************************************/
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Ultrasonic sensors
#define TRIG_FRONT_LEFT   48
#define ECHO_FRONT_LEFT   47
#define TRIG_FRONT_RIGHT  40
#define ECHO_FRONT_RIGHT  41
#define TRIG_SIDE_LEFT    33
#define ECHO_SIDE_LEFT    32
#define TRIG_SIDE_RIGHT   30
#define ECHO_SIDE_RIGHT   29

// Motor drivers
#define PWMA 12
#define DIRA1 34
#define DIRA2 35
#define PWMB 8
#define DIRB1 37
#define DIRB2 36
#define PWMC 6
#define DIRC1 43
#define DIRC2 42
#define PWMD 5
#define DIRD1 A4
#define DIRD2 A5

// Arm servos
#define PIN_LOW_ARM 44
#define PIN_CLAMP   45
#define HOME_LOW_ARM 90
#define HOME_CLAMP   90
#define CLAMP_OPEN_ANGLE 30
#define CLAMP_CLOSE_ANGLE 90

#define MOTORA_FWD(p) do{digitalWrite(DIRA1,LOW);digitalWrite(DIRA2,HIGH);analogWrite(PWMA,constrain((p),0,255));}while(0)
#define MOTORA_STOP() do{digitalWrite(DIRA1,LOW);digitalWrite(DIRA2,LOW);analogWrite(PWMA,0);}while(0)
#define MOTORA_BWD(p) do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW);analogWrite(PWMA,constrain((p),0,255));}while(0)

#define MOTORB_FWD(p) do{digitalWrite(DIRB1,LOW);digitalWrite(DIRB2,HIGH);analogWrite(PWMB,constrain((p),0,255));}while(0)
#define MOTORB_STOP() do{digitalWrite(DIRB1,LOW);digitalWrite(DIRB2,LOW);analogWrite(PWMB,0);}while(0)
#define MOTORB_BWD(p) do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW);analogWrite(PWMB,constrain((p),0,255));}while(0)

#define MOTORC_FWD(p) do{digitalWrite(DIRC1,LOW);digitalWrite(DIRC2,HIGH);analogWrite(PWMC,constrain((p),0,255));}while(0)
#define MOTORC_STOP() do{digitalWrite(DIRC1,LOW);digitalWrite(DIRC2,LOW);analogWrite(PWMC,0);}while(0)
#define MOTORC_BWD(p) do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW);analogWrite(PWMC,constrain((p),0,255));}while(0)

#define MOTORD_FWD(p) do{digitalWrite(DIRD1,LOW);digitalWrite(DIRD2,HIGH);analogWrite(PWMD,constrain((p),0,255));}while(0)
#define MOTORD_STOP() do{digitalWrite(DIRD1,LOW);digitalWrite(DIRD2,LOW);analogWrite(PWMD,0);}while(0)
#define MOTORD_BWD(p) do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW);analogWrite(PWMD,constrain((p),0,255));}while(0)

Servo lowArmServo;
Servo clampServo;

// Motion timing
long TIME_FORWARD_10CM = 500;
long TIME_STRAFE_10CM  = 400;
long TIME_ROTATE_180   = 2050;

// Motor PWM
int PWM_FORWARD = 70;
int PWM_PID      = 55;
int PWM_STRAFE   = 100;
int PWM_ROTATE   = 70;

// Wheel compensation: independent for FWD / BWD
int FWD_LEFT_ADJ   = 4;
int FWD_RIGHT_ADJ  = 0;
int BWD_LEFT_ADJ   = 8;
int BWD_RIGHT_ADJ  = 0;

// Legacy compat: SET:LEFT_ADJ sets both FWD and BWD
int LEFT_WHEEL_ADJUST   = 8;
int RIGHT_WHEEL_ADJUST  = 0;

// Strafe compensation
int FRONT_STRAFE_ADJUST = 40;
int REAR_STRAFE_ADJUST  = 0;

// Arm
int GRIP_OPEN  = CLAMP_OPEN_ANGLE;
int GRIP_CLOSE = CLAMP_CLOSE_ANGLE;
int ARM_SETTLE_MS = 250;

// Alignment / photo
int WALL_TOL = 0;
int FRONT_DIST_TOL = 1;
int ALIGN_SAMPLE_MS = 250;
int PHOTO_SETTLE_MS = 2200;
int CONTINUE_TIMEOUT_MS = 15000;
int MAX_ALIGN_ITER = 80;
int MAX_FRONT_ITER = 60;

// PID params (scaled by x100 for integer upload)
float wallKp = 8.0;
float wallKi = 0.15;
float wallKd = 3.0;
float wallIntegral = 0.0;
float wallPrevError = 0.0;
unsigned long wallPrevTime = 0;
float wallIntegralLimit = 80.0;
int wallPidMinPulse = 18;
int wallPidMaxPulse = 120;

float frontKp = 3.0;
float frontKi = 0.20;
float frontKd = 1.5;
float frontIntegral = 0.0;
float frontPrevError = 0.0;
unsigned long frontPrevTime = 0;
float frontIntegralLimit = 60.0;
int frontPidMinPulse = 20;
int frontPidMaxPulse = 140;

long distFL = 999, distFR = 999, distSL = 999, distSR = 999;
int currentLowArm = HOME_LOW_ARM;
int currentClamp = HOME_CLAMP;

String detectedTrashType = "";

String inputBuffer = "";
bool commandReady = false;

void displayOLED(String a, String b, String c) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println(a);
  display.setCursor(0, 12); display.println(b);
  display.setCursor(0, 24); display.println(c);
  display.display();
}

void stopAll() {
  MOTORA_STOP(); MOTORB_STOP(); MOTORC_STOP(); MOTORD_STOP();
}

void writeLowArm(int angle) {
  angle = constrain(angle, 0, 180);
  lowArmServo.write(angle);
  currentLowArm = angle;
}

void writeClamp(int angle) {
  angle = constrain(angle, 0, 180);
  clampServo.write(angle);
  currentClamp = angle;
}

void armHome() {
  writeLowArm(HOME_LOW_ARM);
  writeClamp(HOME_CLAMP);
  delay(ARM_SETTLE_MS);
  Serial.println("LOG:ARMHOME 90/90");
}

void armInit() {
  lowArmServo.attach(PIN_LOW_ARM);
  delay(150);
  clampServo.attach(PIN_CLAMP);
  delay(150);
  armHome();
}

// ================================================================
// Motion: uses independent FWD / BWD wheel compensation
// ================================================================
void moveForward(int pwm) {
  int lp = pwm + FWD_LEFT_ADJ;
  int rp = pwm + FWD_RIGHT_ADJ;
  MOTORA_FWD(lp); MOTORB_BWD(rp);
  MOTORC_FWD(lp); MOTORD_BWD(rp);
}

void moveBackward(int pwm) {
  int lp = pwm + BWD_LEFT_ADJ;
  int rp = pwm + BWD_RIGHT_ADJ;
  MOTORA_BWD(lp); MOTORB_FWD(rp);
  MOTORC_BWD(lp); MOTORD_FWD(rp);
}

void moveLeft(int pwm) {
  int fp = pwm + FRONT_STRAFE_ADJUST;
  int rp = pwm + REAR_STRAFE_ADJUST;
  MOTORA_BWD(fp); MOTORB_BWD(fp);
  MOTORC_FWD(rp); MOTORD_FWD(rp);
}

void moveRight(int pwm) {
  int fp = pwm + FRONT_STRAFE_ADJUST;
  int rp = pwm + REAR_STRAFE_ADJUST;
  MOTORA_FWD(fp); MOTORB_FWD(fp);
  MOTORC_BWD(rp); MOTORD_BWD(rp);
}

void rotateCW(int pwm) {
  MOTORA_FWD(pwm); MOTORB_FWD(pwm);
  MOTORC_FWD(pwm); MOTORD_FWD(pwm);
}

void rotateCCW(int pwm) {
  MOTORA_BWD(pwm); MOTORB_BWD(pwm);
  MOTORC_BWD(pwm); MOTORD_BWD(pwm);
}

// Timed motion helpers
void timedForwardCustom(int pwm, long ms) { moveForward(pwm); delay(ms); stopAll(); }
void timedBackwardCustom(int pwm, long ms) { moveBackward(pwm); delay(ms); stopAll(); }
void timedStrafeLeftCustom(int pwm, long ms) { moveLeft(pwm); delay(ms); stopAll(); }
void timedStrafeRightCustom(int pwm, long ms) { moveRight(pwm); delay(ms); stopAll(); }

// ================================================================
// Ultrasonic distance measurement
// ================================================================
long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999;
  return (long)((duration / 2.0) / 29.1);
}

void updateDistances() {
  distFL = measureDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT); delay(5);
  distFR = measureDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT); delay(5);
  distSL = measureDistance(TRIG_SIDE_LEFT, ECHO_SIDE_LEFT); delay(5);
  distSR = measureDistance(TRIG_SIDE_RIGHT, ECHO_SIDE_RIGHT);
}

void sampleFrontDistances(unsigned long sampleMs) {
  stopAll();
  unsigned long start = millis();
  long sumFL = 0, sumFR = 0;
  int cntFL = 0, cntFR = 0;

  while (millis() - start < sampleMs) {
    long fl = measureDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT); delay(3);
    long fr = measureDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT); delay(3);
    if (fl != 999) { sumFL += fl; cntFL++; }
    if (fr != 999) { sumFR += fr; cntFR++; }
  }

  distFL = (cntFL > 0) ? (sumFL / cntFL) : 999;
  distFR = (cntFR > 0) ? (sumFR / cntFR) : 999;
}

float getAvgFront() {
  if (distFL == 999 && distFR == 999) return 999;
  if (distFL == 999) return distFR;
  if (distFR == 999) return distFL;
  return (distFL + distFR) / 2.0;
}

// ================================================================
// PID computation
// ================================================================
void resetWallPID() {
  wallIntegral = 0;
  wallPrevError = 0;
  wallPrevTime = 0;
}

void resetFrontPID() {
  frontIntegral = 0;
  frontPrevError = 0;
  frontPrevTime = 0;
}

int computeWallPIDPulse(float error) {
  unsigned long now = millis();
  float dt = (wallPrevTime == 0) ? 0.05 : (now - wallPrevTime) / 1000.0;
  if (dt <= 0) dt = 0.05;
  if (dt > 0.30) dt = 0.30;

  wallIntegral += error * dt;
  if (wallIntegral > wallIntegralLimit) wallIntegral = wallIntegralLimit;
  if (wallIntegral < -wallIntegralLimit) wallIntegral = -wallIntegralLimit;

  float derivative = (error - wallPrevError) / dt;
  float output = wallKp * error + wallKi * wallIntegral + wallKd * derivative;

  wallPrevError = error;
  wallPrevTime = now;

  int pulse = (int)fabs(output);
  pulse = constrain(pulse, wallPidMinPulse, wallPidMaxPulse);
  return pulse;
}

int computeFrontPIDPulse(float error) {
  unsigned long now = millis();
  float dt = (frontPrevTime == 0) ? 0.05 : (now - frontPrevTime) / 1000.0;
  if (dt <= 0) dt = 0.05;
  if (dt > 0.30) dt = 0.30;

  frontIntegral += error * dt;
  if (frontIntegral > frontIntegralLimit) frontIntegral = frontIntegralLimit;
  if (frontIntegral < -frontIntegralLimit) frontIntegral = -frontIntegralLimit;

  float derivative = (error - frontPrevError) / dt;
  float output = frontKp * error + frontKi * frontIntegral + frontKd * derivative;

  frontPrevError = error;
  frontPrevTime = now;

  int pulse = (int)fabs(output);
  pulse = constrain(pulse, frontPidMinPulse, frontPidMaxPulse);
  return pulse;
}

// ================================================================
// PID wall alignment (WALIGN)
// ================================================================
bool alignToWallPID() {
  resetWallPID();

  for (int i = 0; i < MAX_ALIGN_ITER; i++) {
    sampleFrontDistances(ALIGN_SAMPLE_MS);

    if (distFL == 999 || distFR == 999) {
      Serial.println("LOG:ALIGN_NO_READING");
      delay(30);
      continue;
    }

    int error = (int)distFL - (int)distFR;
    Serial.print("LOG:ALIGN iter=");
    Serial.print(i);
    Serial.print(" FL=");
    Serial.print(distFL);
    Serial.print(" FR=");
    Serial.print(distFR);
    Serial.print(" err=");
    Serial.println(error);

    if (abs(error) <= WALL_TOL) {
      Serial.println("LOG:WALL_ALIGNED");
      stopAll();
      return true;
    }

    int pulse = computeWallPIDPulse((float)error);

    if (error > 0) rotateCW(PWM_ROTATE);
    else rotateCCW(PWM_ROTATE);

    delay(pulse);
    stopAll();
    delay(35);
  }

  Serial.println("LOG:ALIGN_TIMEOUT");
  stopAll();
  return false;
}

// ================================================================
// PID front distance control (FRONTPID) — core ultrasonic stepping
// Forward uses FWD_LEFT_ADJ / FWD_RIGHT_ADJ
// Backward uses BWD_LEFT_ADJ / BWD_RIGHT_ADJ
// ================================================================
bool moveFrontToDistancePID(float targetCm) {
  resetFrontPID();

  for (int i = 0; i < MAX_FRONT_ITER; i++) {
    sampleFrontDistances(ALIGN_SAMPLE_MS);
    float avgFront = getAvgFront();

    if (avgFront == 999) {
      Serial.println("LOG:FRONTPID_NO_READING");
      delay(40);
      continue;
    }

    float error = avgFront - targetCm;
    Serial.print("LOG:FRONTPID iter=");
    Serial.print(i);
    Serial.print(" AVG=");
    Serial.print(avgFront, 1);
    Serial.print(" target=");
    Serial.print(targetCm, 1);
    Serial.print(" err=");
    Serial.println(error, 1);

    if (fabs(error) <= FRONT_DIST_TOL) {
      Serial.println("LOG:FRONTPID_REACHED");
      stopAll();
      return true;
    }

    int pulse = computeFrontPIDPulse(error);

    // Use different wheel comp based on direction
    if (error > 0) moveForward(PWM_PID);
    else moveBackward(PWM_PID);

    delay(pulse);
    stopAll();
    delay(40);
  }

  Serial.println("LOG:FRONTPID_TIMEOUT");
  stopAll();
  return false;
}

// ================================================================
// Timed rotation (no gyroscope)
// ================================================================
void rotateTimed(float angleDeg) {
  if (angleDeg == 0) {
    Serial.println("LOG:ROT_SKIP_0");
    return;
  }

  long rotTime = (long)(abs(angleDeg) / 180.0 * TIME_ROTATE_180);

  Serial.print("LOG:ROT_TIMED angle=");
  Serial.print(angleDeg, 1);
  Serial.print(" time_ms=");
  Serial.println(rotTime);

  if (angleDeg > 0) rotateCW(PWM_ROTATE);
  else rotateCCW(PWM_ROTATE);

  delay(rotTime);
  stopAll();
  Serial.println("LOG:ROT_DONE");
}

// ================================================================
// Wait for Pi to reply CONTINUE / STOP
// Also executes motion commands from Pi (SL/SR/WALIGN etc.)
// so Pi can control robot movement during WAIT:CAM for centering
// ================================================================
bool waitForContinueOrStop(const char* context) {
  unsigned long deadline = millis() + CONTINUE_TIMEOUT_MS;
  while (millis() < deadline) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;

      if (line == "CONTINUE") {
        Serial.println("LOG:CONTINUE_RX");
        return true;
      }
      if (line == "STOP" || line == "ABORT") {
        Serial.print("LOG:STOP_RX during ");
        Serial.println(context);
        return false;
      }
      if (line.startsWith("TYPE:")) {
        detectedTrashType = line.substring(5);
        detectedTrashType.trim();
        detectedTrashType.toUpperCase();
        Serial.print("LOG:TYPE_RX:");
        Serial.println(detectedTrashType);
        continue;
      }

      // Execute motion commands from Pi (for centering)
      // These run immediately, then continue waiting for CONTINUE
      bool executed = false;

      if (line.startsWith("SL:")) {
        int sep = line.indexOf(':', 3);
        if (sep > 0) {
          int pwm = line.substring(3, sep).toInt();
          long ms = line.substring(sep + 1).toInt();
          timedStrafeLeftCustom(pwm, ms);
          Serial.print("LOG:WAIT_EXEC_SL:");
          Serial.println(ms);
          executed = true;
        }
      }
      else if (line.startsWith("SR:")) {
        int sep = line.indexOf(':', 3);
        if (sep > 0) {
          int pwm = line.substring(3, sep).toInt();
          long ms = line.substring(sep + 1).toInt();
          timedStrafeRightCustom(pwm, ms);
          Serial.print("LOG:WAIT_EXEC_SR:");
          Serial.println(ms);
          executed = true;
        }
      }
      else if (line == "WALIGN") {
        alignToWallPID();
        Serial.println("LOG:WAIT_EXEC_WALIGN");
        executed = true;
      }
      else if (line.startsWith("FRONTPID:")) {
        float target = line.substring(9).toFloat();
        moveFrontToDistancePID(target);
        Serial.print("LOG:WAIT_EXEC_FRONTPID:");
        Serial.println(target, 1);
        executed = true;
      }
      else if (line == "STOP") {
        stopAll();
        Serial.println("LOG:WAIT_EXEC_STOP");
        executed = true;
      }

      if (!executed) {
        Serial.print("LOG:IGNORED_DURING_");
        Serial.print(context);
        Serial.print(":");
        Serial.println(line);
      }
    }
    delay(10);
  }
  Serial.print("LOG:CONTINUE_TIMEOUT during ");
  Serial.println(context);
  return false;
}

void waitForContinue() {
  unsigned long deadline = millis() + CONTINUE_TIMEOUT_MS;
  while (millis() < deadline) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();
      if (line == "CONTINUE") {
        Serial.println("LOG:CONTINUE_RX");
        return;
      }
      if (line.length() > 0) {
        Serial.print("LOG:IGNORED_DURING_WAIT:");
        Serial.println(line);
      }
    }
    delay(10);
  }
  Serial.println("LOG:CONTINUE_TIMEOUT");
}

// ================================================================
// Photo-sync strafe (SHOTL / SHOTR)
// ================================================================
void executePhotoMove(bool moveLeftDir, int pwm, long totalMs) {
  const int totalSteps = 20;
  long stepMs = totalMs / totalSteps;
  if (stepMs < 1) stepMs = 1;

  for (int step = 1; step <= totalSteps; step++) {
    if (moveLeftDir) moveLeft(pwm);
    else moveRight(pwm);

    delay(stepMs);
    stopAll();

    Serial.print("LOG:STEP_STOPPED ");
    Serial.print(step);
    Serial.print("/");
    Serial.println(totalSteps);

    Serial.print("LOG:STEP_ALIGN ");
    Serial.print(step);
    Serial.print("/");
    Serial.println(totalSteps);
    alignToWallPID();

    delay(PHOTO_SETTLE_MS);

    Serial.print("WAIT_SHOT:");
    Serial.print(moveLeftDir ? "LEFT" : "RIGHT");
    Serial.print(":");
    Serial.print(step);
    Serial.print("/");
    Serial.println(totalSteps);

    waitForContinue();
  }

  Serial.println(moveLeftDir ? "DONE:SHOTL" : "DONE:SHOTR");
}

// ================================================================
// Sorting sequence execution
// ================================================================
void executeSequence(String trashType) {
  trashType.toUpperCase();

  detectedTrashType = "";

  // ============ Phase 1: Positioning & Pickup ============

  moveFrontToDistancePID(25.0);
  Serial.println("LOG:SEQ_FRONTPID25");

  alignToWallPID();
  Serial.println("LOG:SEQ_WALIGN1");

  Serial.println("WAIT:CAM");
  if (!waitForContinueOrStop("SEQ_CAM")) {
    stopAll();
    Serial.println("DONE:SEQ:ABORTED");
    return;
  }

  if (detectedTrashType.length() > 0) {
    Serial.print("LOG:SEQ_USING_DETECTED_TYPE:");
    Serial.println(detectedTrashType);
    trashType = detectedTrashType;
  }

  writeClamp(GRIP_OPEN);
  delay(ARM_SETTLE_MS);
  Serial.println("LOG:SEQ_GRIP_OPEN");

  moveFrontToDistancePID(6.0);
  Serial.println("LOG:SEQ_FRONTPID6");

  writeClamp(GRIP_CLOSE);
  delay(ARM_SETTLE_MS);
  Serial.println("LOG:SEQ_GRAB");

  // ============ Phase 2: Transport to Drop Zone ============

  moveFrontToDistancePID(30.0);
  Serial.println("LOG:SEQ_FRONTPID30");

  rotateTimed(180.0);
  Serial.println("LOG:SEQ_ROT1");

  writeLowArm(45);
  delay(500);
  Serial.println("LOG:SEQ_ARM_DOWN45");

  moveFrontToDistancePID(5.0);
  Serial.println("LOG:SEQ_FRONTPID5");

  // ============ Phase 3: Drop & Return ============

  if (trashType == "METAL") {
    timedStrafeLeftCustom(PWM_STRAFE, (TIME_STRAFE_10CM * 7) / 10);
    Serial.println("LOG:SEQ_LEFT7");

    writeClamp(GRIP_OPEN);
    delay(ARM_SETTLE_MS);
    Serial.println("LOG:SEQ_RELEASE");

    moveFrontToDistancePID(15.0);
    Serial.println("LOG:SEQ_FRONTPID15");

    timedStrafeRightCustom(PWM_STRAFE, (TIME_STRAFE_10CM * 7) / 10);
    Serial.println("LOG:SEQ_RIGHT7");
  }
  else if (trashType == "PLASTIC") {
    timedStrafeRightCustom(PWM_STRAFE, (TIME_STRAFE_10CM * 7) / 10);
    Serial.println("LOG:SEQ_RIGHT7");

    writeClamp(GRIP_OPEN);
    delay(ARM_SETTLE_MS);
    Serial.println("LOG:SEQ_RELEASE");

    moveFrontToDistancePID(15.0);
    Serial.println("LOG:SEQ_FRONTPID15");

    timedStrafeLeftCustom(PWM_STRAFE, (TIME_STRAFE_10CM * 7) / 10);
    Serial.println("LOG:SEQ_LEFT7");
  }

  writeLowArm(HOME_LOW_ARM);
  delay(500);
  Serial.println("LOG:SEQ_ARM_HOME");

  alignToWallPID();
  Serial.println("LOG:SEQ_WALIGN2");

  rotateTimed(180.0);
  Serial.println("LOG:SEQ_ROT2");

  alignToWallPID();
  Serial.println("LOG:SEQ_WALIGN3");

  moveFrontToDistancePID(25.0);
  Serial.println("LOG:SEQ_FRONTPID25_RETURN");

  Serial.println("DONE:SEQ:OK");
}

// ================================================================
// Parameter set / get
// ================================================================
bool setParameter(String param, long value) {
  if (param == "TIME_FWD_10CM") TIME_FORWARD_10CM = value;
  else if (param == "TIME_STRAFE_10CM") TIME_STRAFE_10CM = value;
  else if (param == "TIME_ROTATE_180") TIME_ROTATE_180 = value;
  else if (param == "PWM_FWD") PWM_FORWARD = (int)value;
  else if (param == "PWM_PID") PWM_PID = (int)value;
  else if (param == "PWM_STRAFE") PWM_STRAFE = (int)value;
  else if (param == "PWM_ROTATE") PWM_ROTATE = (int)value;

  // Independent FWD/BWD wheel comp
  else if (param == "FWD_LEFT_ADJ")  FWD_LEFT_ADJ  = (int)value;
  else if (param == "FWD_RIGHT_ADJ") FWD_RIGHT_ADJ = (int)value;
  else if (param == "BWD_LEFT_ADJ")  BWD_LEFT_ADJ  = (int)value;
  else if (param == "BWD_RIGHT_ADJ") BWD_RIGHT_ADJ = (int)value;

  // Legacy: SET:LEFT_ADJ sets both FWD and BWD
  else if (param == "LEFT_ADJ") {
    LEFT_WHEEL_ADJUST = (int)value;
    FWD_LEFT_ADJ  = (int)value;
    BWD_LEFT_ADJ  = (int)value;
  }
  else if (param == "RIGHT_ADJ") {
    RIGHT_WHEEL_ADJUST = (int)value;
    FWD_RIGHT_ADJ = (int)value;
    BWD_RIGHT_ADJ = (int)value;
  }

  else if (param == "FRONT_STRAFE_ADJ") FRONT_STRAFE_ADJUST = (int)value;
  else if (param == "REAR_STRAFE_ADJ") REAR_STRAFE_ADJUST = (int)value;
  else if (param == "WALL_TOL") WALL_TOL = (int)value;
  else if (param == "FRONT_DIST_TOL") FRONT_DIST_TOL = (int)value;
  else if (param == "ALIGN_SAMPLE_MS") ALIGN_SAMPLE_MS = (int)value;
  else if (param == "PHOTO_SETTLE_MS") PHOTO_SETTLE_MS = (int)value;
  else if (param == "ARM_SETTLE_MS") ARM_SETTLE_MS = (int)value;
  else if (param == "MAX_ALIGN_ITER") MAX_ALIGN_ITER = (int)value;
  else if (param == "MAX_FRONT_ITER") MAX_FRONT_ITER = (int)value;
  else if (param == "CONTINUE_TIMEOUT_MS") CONTINUE_TIMEOUT_MS = (int)value;
  else if (param == "GRIP_OPEN") GRIP_OPEN = (int)value;
  else if (param == "GRIP_CLOSE") GRIP_CLOSE = (int)value;
  else if (param == "WALL_KP_X100") wallKp = value / 100.0;
  else if (param == "WALL_KI_X100") wallKi = value / 100.0;
  else if (param == "WALL_KD_X100") wallKd = value / 100.0;
  else if (param == "FRONT_KP_X100") frontKp = value / 100.0;
  else if (param == "FRONT_KI_X100") frontKi = value / 100.0;
  else if (param == "FRONT_KD_X100") frontKd = value / 100.0;
  else if (param == "WALL_PID_MIN") wallPidMinPulse = (int)value;
  else if (param == "WALL_PID_MAX") wallPidMaxPulse = (int)value;
  else if (param == "FRONT_PID_MIN") frontPidMinPulse = (int)value;
  else if (param == "FRONT_PID_MAX") frontPidMaxPulse = (int)value;
  else return false;
  return true;
}

void sendParameter(String param) {
  Serial.print("DATA:");
  Serial.print(param);
  Serial.print(":");

  if (param == "TIME_FWD_10CM") Serial.println(TIME_FORWARD_10CM);
  else if (param == "TIME_STRAFE_10CM") Serial.println(TIME_STRAFE_10CM);
  else if (param == "TIME_ROTATE_180") Serial.println(TIME_ROTATE_180);
  else if (param == "PWM_FWD") Serial.println(PWM_FORWARD);
  else if (param == "PWM_PID") Serial.println(PWM_PID);
  else if (param == "PWM_STRAFE") Serial.println(PWM_STRAFE);
  else if (param == "PWM_ROTATE") Serial.println(PWM_ROTATE);
  else if (param == "FWD_LEFT_ADJ")  Serial.println(FWD_LEFT_ADJ);
  else if (param == "FWD_RIGHT_ADJ") Serial.println(FWD_RIGHT_ADJ);
  else if (param == "BWD_LEFT_ADJ")  Serial.println(BWD_LEFT_ADJ);
  else if (param == "BWD_RIGHT_ADJ") Serial.println(BWD_RIGHT_ADJ);
  else if (param == "LEFT_ADJ") Serial.println(FWD_LEFT_ADJ);
  else if (param == "RIGHT_ADJ") Serial.println(FWD_RIGHT_ADJ);
  else if (param == "FRONT_STRAFE_ADJ") Serial.println(FRONT_STRAFE_ADJUST);
  else if (param == "REAR_STRAFE_ADJ") Serial.println(REAR_STRAFE_ADJUST);
  else if (param == "WALL_TOL") Serial.println(WALL_TOL);
  else if (param == "FRONT_DIST_TOL") Serial.println(FRONT_DIST_TOL);
  else if (param == "ALIGN_SAMPLE_MS") Serial.println(ALIGN_SAMPLE_MS);
  else if (param == "PHOTO_SETTLE_MS") Serial.println(PHOTO_SETTLE_MS);
  else if (param == "ARM_SETTLE_MS") Serial.println(ARM_SETTLE_MS);
  else if (param == "MAX_ALIGN_ITER") Serial.println(MAX_ALIGN_ITER);
  else if (param == "MAX_FRONT_ITER") Serial.println(MAX_FRONT_ITER);
  else if (param == "CONTINUE_TIMEOUT_MS") Serial.println(CONTINUE_TIMEOUT_MS);
  else if (param == "GRIP_OPEN") Serial.println(GRIP_OPEN);
  else if (param == "GRIP_CLOSE") Serial.println(GRIP_CLOSE);
  else if (param == "LOWARM") Serial.println(currentLowArm);
  else if (param == "CLAMP") Serial.println(currentClamp);
  else Serial.println("ERR");
}

// ================================================================
// Command parser
// ================================================================
void executeCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  cmd.toUpperCase();

  // Basic motion
  if (cmd.startsWith("FWD:")) {
    int sep = cmd.indexOf(':', 4);
    if (sep > 0) {
      int pwm = cmd.substring(4, sep).toInt();
      long ms = cmd.substring(sep + 1).toInt();
      Serial.println("OK:FWD");
      timedForwardCustom(pwm, ms);
      Serial.println("DONE:FWD");
    } else Serial.println("ERR:Bad FWD");
  }
  else if (cmd.startsWith("BWD:")) {
    int sep = cmd.indexOf(':', 4);
    if (sep > 0) {
      int pwm = cmd.substring(4, sep).toInt();
      long ms = cmd.substring(sep + 1).toInt();
      Serial.println("OK:BWD");
      timedBackwardCustom(pwm, ms);
      Serial.println("DONE:BWD");
    } else Serial.println("ERR:Bad BWD");
  }
  else if (cmd.startsWith("SL:")) {
    int sep = cmd.indexOf(':', 3);
    if (sep > 0) {
      int pwm = cmd.substring(3, sep).toInt();
      long ms = cmd.substring(sep + 1).toInt();
      Serial.println("OK:SL");
      timedStrafeLeftCustom(pwm, ms);
      Serial.println("DONE:SL");
    } else Serial.println("ERR:Bad SL");
  }
  else if (cmd.startsWith("SR:")) {
    int sep = cmd.indexOf(':', 3);
    if (sep > 0) {
      int pwm = cmd.substring(3, sep).toInt();
      long ms = cmd.substring(sep + 1).toInt();
      Serial.println("OK:SR");
      timedStrafeRightCustom(pwm, ms);
      Serial.println("DONE:SR");
    } else Serial.println("ERR:Bad SR");
  }

  // Rotation
  else if (cmd.startsWith("ROTC:")) {
    int sep = cmd.indexOf(':', 5);
    if (sep > 0) {
      int pwm = cmd.substring(5, sep).toInt();
      long ms = cmd.substring(sep + 1).toInt();
      Serial.println("OK:ROTC");
      rotateCW(pwm);
      delay(ms);
      stopAll();
      Serial.println("DONE:ROTC");
    } else Serial.println("ERR:Bad ROTC");
  }
  else if (cmd.startsWith("ROTCCW:")) {
    int sep = cmd.indexOf(':', 7);
    if (sep > 0) {
      int pwm = cmd.substring(7, sep).toInt();
      long ms = cmd.substring(sep + 1).toInt();
      Serial.println("OK:ROTCCW");
      rotateCCW(pwm);
      delay(ms);
      stopAll();
      Serial.println("DONE:ROTCCW");
    } else Serial.println("ERR:Bad ROTCCW");
  }
  else if (cmd.startsWith("ROT:")) {
    float angle = cmd.substring(4).toFloat();
    Serial.println("OK:ROT");
    rotateTimed(angle);
    Serial.println("DONE:ROT");
  }

  // Photo-sync strafe
  else if (cmd.startsWith("SHOTL:")) {
    int sep = cmd.indexOf(':', 6);
    if (sep > 0) {
      int pwm = cmd.substring(6, sep).toInt();
      long ms = cmd.substring(sep + 1).toInt();
      Serial.println("OK:SHOTL");
      executePhotoMove(true, pwm, ms);
    } else Serial.println("ERR:Bad SHOTL");
  }
  else if (cmd.startsWith("SHOTR:")) {
    int sep = cmd.indexOf(':', 6);
    if (sep > 0) {
      int pwm = cmd.substring(6, sep).toInt();
      long ms = cmd.substring(sep + 1).toInt();
      Serial.println("OK:SHOTR");
      executePhotoMove(false, pwm, ms);
    } else Serial.println("ERR:Bad SHOTR");
  }

  // PID control
  else if (cmd.startsWith("FRONTPID:")) {
    float target = cmd.substring(9).toFloat();
    Serial.println("OK:FRONTPID");
    moveFrontToDistancePID(target);
    Serial.println("DONE:FRONTPID");
  }
  else if (cmd == "WALIGN") {
    Serial.println("OK:WALIGN");
    alignToWallPID();
    Serial.println("DONE:WALIGN");
  }

  // Grab / Release
  else if (cmd == "GRAB") {
    Serial.println("OK:GRAB");
    writeClamp(GRIP_CLOSE);
    delay(ARM_SETTLE_MS);
    Serial.print("DATA:GRIP:");
    Serial.println(currentClamp);
    Serial.println("DONE:GRAB");
  }
  else if (cmd == "RELEASE") {
    Serial.println("OK:RELEASE");
    writeClamp(GRIP_OPEN);
    delay(ARM_SETTLE_MS);
    Serial.print("DATA:GRIP:");
    Serial.println(currentClamp);
    Serial.println("DONE:RELEASE");
  }

  // Sorting sequences
  else if (cmd.startsWith("SEQ:")) {
    String trashType = cmd.substring(4);
    Serial.print("OK:SEQ:");
    Serial.println(trashType);
    if (trashType == "METAL" || trashType == "PLASTIC" || trashType == "AUTO") {
      executeSequence(trashType);
    } else {
      Serial.print("ERR:Unknown SEQ type:");
      Serial.println(trashType);
      Serial.println("DONE:SEQ:ERR");
    }
  }

  // Arm
  else if (cmd == "ARMHOME") {
    armHome();
    Serial.println("DONE:ARMHOME");
  }
  else if (cmd.startsWith("LOWARM:")) {
    int angle = cmd.substring(7).toInt();
    writeLowArm(angle);
    Serial.print("DATA:LOWARM:");
    Serial.println(currentLowArm);
  }
  else if (cmd.startsWith("GRIP:")) {
    int angle = cmd.substring(5).toInt();
    writeClamp(angle);
    Serial.print("DATA:GRIP:");
    Serial.println(currentClamp);
  }

  // Sensors / utilities
  else if (cmd == "DIST") {
    updateDistances();
    Serial.print("DATA:FL:"); Serial.print(distFL);
    Serial.print(":FR:");     Serial.print(distFR);
    Serial.print(":SL:");     Serial.print(distSL);
    Serial.print(":SR:");     Serial.println(distSR);
  }
  else if (cmd == "STOP") {
    stopAll();
    Serial.println("DONE:STOP");
  }
  else if (cmd.startsWith("SET:")) {
    int sep = cmd.indexOf(':', 4);
    if (sep > 0) {
      String param = cmd.substring(4, sep);
      long value = cmd.substring(sep + 1).toInt();
      if (setParameter(param, value)) {
        Serial.print("OK:SET:");
        Serial.print(param);
        Serial.print(":");
        Serial.println(value);
      } else {
        Serial.print("ERR:Unknown param:");
        Serial.println(param);
      }
    } else Serial.println("ERR:Bad SET");
  }
  else if (cmd.startsWith("GET:")) {
    sendParameter(cmd.substring(4));
  }
  else if (cmd == "PING") {
    Serial.println("PONG");
  }
  else if (cmd == "CONTINUE") {
    Serial.println("LOG:CONTINUE_IGNORED_IDLE");
  }
  else {
    Serial.print("ERR:Unknown command:");
    Serial.println(cmd);
  }
}

// ================================================================
// Setup / Loop
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  displayOLED("Booting...", "Unified v2c", "FWD/BWD+WALIGN");

  pinMode(PWMA, OUTPUT); pinMode(DIRA1, OUTPUT); pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(DIRB1, OUTPUT); pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT); pinMode(DIRC1, OUTPUT); pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT); pinMode(DIRD1, OUTPUT); pinMode(DIRD2, OUTPUT);

  pinMode(TRIG_FRONT_LEFT, OUTPUT);  pinMode(ECHO_FRONT_LEFT, INPUT);
  pinMode(TRIG_FRONT_RIGHT, OUTPUT); pinMode(ECHO_FRONT_RIGHT, INPUT);
  pinMode(TRIG_SIDE_LEFT, OUTPUT);   pinMode(ECHO_SIDE_LEFT, INPUT);
  pinMode(TRIG_SIDE_RIGHT, OUTPUT);  pinMode(ECHO_SIDE_RIGHT, INPUT);

  stopAll();

  Wire.begin();

  armInit();
  updateDistances();

  displayOLED("READY", "115200", "v2c AlignFix");
  Serial.println("READY");
  Serial.println("LOG:Unified v2c ready (Phase2 WALIGN fix + FWD/BWD adj)");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) commandReady = true;
    } else {
      inputBuffer += c;
    }
  }

  if (commandReady) {
    inputBuffer.trim();
    executeCommand(inputBuffer);
    inputBuffer = "";
    commandReady = false;
  }

  delay(10);
}
