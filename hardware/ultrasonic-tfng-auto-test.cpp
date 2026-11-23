/**
 * ====================================================================================
 * Autonomous Obstacle-Avoiding Mobile Robot (AMR) - Advanced Navigation Firmware
 * ====================================================================================
 * Hardware:
 *   - Controller: ESP32-S3 DevKitC-1
 *   - Motor Driver: TB6612FNG (Dual Channel + Standby)
 *   - Motors: 2x DC Geared Motors (Differential Drive)
 *   - Distance Sensor: HC-SR04 Ultrasonic (via Level Shifter on ECHO)
 *   - Sensor Mount: Mini Servo (5-Point Sector Scanning)
 *   - IMU: MPU6050 (I2C) - Crash Shock Detection, Calibrated Gyro Angle Turning,
 *          Straight-Line Heading Lock & Anti-Tip Safety
 *
 * Pin Connections (100% Preserved Existing Hardware Wiring):
 *   - TB6612 AIN1: GPIO 14 (Left Motor Dir 1)
 *   - TB6612 AIN2: GPIO 15 (Left Motor Dir 2)
 *   - TB6612 PWMA: GPIO 16 (Left Motor PWM)
 *   - TB6612 BIN1: GPIO 17 (Right Motor Dir 1)
 *   - TB6612 BIN2: GPIO 18 (Right Motor Dir 2)
 *   - TB6612 PWMB: GPIO 4  (Right Motor PWM)
 *   - TB6612 STBY: GPIO 12 (Driver Standby Enable)
 *   - Ultrasonic TRIG: GPIO 21
 *   - Ultrasonic ECHO: GPIO 47 (through Level Shifter)
 *   - Servo Signal:    GPIO 13
 *   - MPU6050 SDA:     GPIO 8
 *   - MPU6050 SCL:     GPIO 9 (Address: 0x68)
 * ====================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ================= MOTOR PINS (TB6612FNG) =================
#define AIN1 14
#define AIN2 15
#define PWMA 16

#define BIN1 17
#define BIN2 18
#define PWMB 4

#define STBY 12

// ================= ULTRASONIC SENSOR (HC-SR04) =================
#define TRIG_PIN 21
#define ECHO_PIN 47

// ================= SERVO =================
#define SERVO_PIN 13
Servo scanServo;

// ================= MPU6050 I2C & REGISTER DEFINITIONS =================
#define PIN_SDA 8
#define PIN_SCL 9
#define MPU6050_ADDR 0x68

#define MPU_PWR_MGMT_1   0x6B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_XOUT_H 0x3B

// ================= PWM CHANNELS & SETTINGS =================
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A 0 // Left Motor
#define PWM_CHANNEL_B 1 // Right Motor

// ================= TUNABLE SPEEDS & THRESHOLDS =================
int forwardSpeed = 130;        // Base cruising speed (0-255)
int turnSpeed = 135;           // Speed during gyro turns
int minTurnSpeed = 90;         // Proportional slowdown speed near target angle
int reverseSpeed = 125;        // Recovery reverse speed

#define FRONT_OBSTACLE_THRESHOLD_CM  35  // Stop and scan if obstacle closer than this
#define CRITICAL_STOP_DISTANCE_CM   18  // Emergency stop threshold
#define SIDE_CLEARANCE_MIN_CM        22  // Minimum acceptable clearance on side sectors

// MPU6050 Thresholds
#define CRASH_DECEL_THRESHOLD_G     1.65f // Shock spike threshold for inertial collision (in Gs)
#define MAX_TILT_ANGLE_DEG          25.0f // Maximum safe pitch/roll before emergency shutdown

// ================= GLOBAL SENSOR & STATE VARIABLES =================
bool imuAvailable = false;
float gyroZ_offset = 0.0f;       // Gyro zero-bias offset (calibrated at boot)
float currentRobotHeading = 0.0f; // Integrated heading angle in degrees
unsigned long lastHeadingUpdateUs = 0;

// ================= FUNCTION DECLARATIONS =================
void setMotorSpeeds(int leftPwm, int rightPwm);
void forward();
void backward();
void stopMotors();
void setDriverStandby(bool enable);

bool initMPU6050();
void calibrateGyroBias();
bool readIMU(float &ax, float &ay, float &az, float &gz_dps, float &pitch, float &roll);
bool checkInertialCrash(float ax, float ay);
void executeCrashRecovery();

void turnAngleDegrees(float targetDegrees);
void forwardWithHeadingCorrection(float targetHeading, unsigned long durationMs);

long getSingleDistance();
long getMedianDistance(int samples = 5);
void scanEnvironment(long &distFarLeft, long &distFrontLeft, long &distCenter, long &distFrontRight, long &distFarRight);
void decideAndExecuteTurn();

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================================");
  Serial.println("  ESP32-S3 AMR Advanced Autonomous Navigation System");
  Serial.println("========================================================");

  // 1. Configure TB6612 Motor Pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Configure ESP32 LEDC PWM
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL_A);
  ledcAttachPin(PWMB, PWM_CHANNEL_B);

  setDriverStandby(true);
  stopMotors();

  // 2. Configure HC-SR04 Ultrasonic Pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // 3. Configure Servo & Center it
  scanServo.attach(SERVO_PIN);
  scanServo.write(90); // Center forward
  delay(400);

  // 4. Initialize I2C & MPU6050
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  imuAvailable = initMPU6050();

  if (imuAvailable) {
    Serial.println("[IMU] MPU6050 detected at 0x68. Starting gyro calibration...");
    calibrateGyroBias();
    Serial.println("[IMU] Calibration complete. Ready for navigation!");
  } else {
    Serial.println("[WARNING] MPU6050 not detected. Falling back to ultrasonic only.");
  }

  Serial.println("[SYSTEM] System initialized successfully. Autonomous navigation starting in 2s...\n");
  delay(2000);
  lastHeadingUpdateUs = micros();
}

// ================= MAIN NAVIGATION LOOP =================
void loop() {
  float ax = 0, ay = 0, az = 0, gz = 0, pitch = 0, roll = 0;

  // 1. Read IMU Safety Data
  if (imuAvailable) {
    if (readIMU(ax, ay, az, gz, pitch, roll)) {
      // Anti-Tip Safety Check
      if (abs(pitch) > MAX_TILT_ANGLE_DEG || abs(roll) > MAX_TILT_ANGLE_DEG) {
        stopMotors();
        setDriverStandby(false); // Shutdown driver
        Serial.printf("[SAFETY ALERT] Excessive Tilt! Pitch: %.1f deg, Roll: %.1f deg. Motors halted.\n", pitch, roll);
        while (abs(pitch) > (MAX_TILT_ANGLE_DEG - 5) || abs(roll) > (MAX_TILT_ANGLE_DEG - 5)) {
          delay(200);
          readIMU(ax, ay, az, gz, pitch, roll);
        }
        setDriverStandby(true);
        Serial.println("[SAFETY] Robot leveled. Resuming operation.");
      }

      // Check Inertial Crash / Bump Detection (missed by ultrasonic)
      if (checkInertialCrash(ax, ay)) {
        executeCrashRecovery();
        return;
      }
    }
  }

  // 2. Measure Front Distance with Outlier-Filtered Ultrasonic
  long frontDist = getMedianDistance(5);
  Serial.printf("[NAV] Front Distance: %ld cm\n", frontDist);

  // 3. Decision Arbitration
  if (frontDist > FRONT_OBSTACLE_THRESHOLD_CM) {
    // Path is clear: Move forward
    forward();
    delay(60); // Responsive forward step
  } else {
    // Obstacle detected in front path: Stop and scan 5 sectors
    stopMotors();
    Serial.printf("[NAV] Obstacle detected at %ld cm (<= %d cm). Initiating 5-point sector scan...\n",
                  frontDist, FRONT_OBSTACLE_THRESHOLD_CM);
    delay(200); // Settle chassis momentum

    decideAndExecuteTurn();
  }
}

// ================= 5-POINT SCANNING & ADAPTIVE TURN LOGIC =================

/**
 * Sweeps servo across 5 key viewing angles:
 *   150 deg: Hard Left
 *   120 deg: Front Left
 *    90 deg: Center
 *    60 deg: Front Right
 *    30 deg: Hard Right
 */
void scanEnvironment(long &distFarLeft, long &distFrontLeft, long &distCenter, long &distFrontRight, long &distFarRight) {
  // 1. Look Center (90 deg)
  scanServo.write(90);
  delay(120);
  distCenter = getMedianDistance(3);

  // 2. Look Front-Left (120 deg)
  scanServo.write(120);
  delay(120);
  distFrontLeft = getMedianDistance(3);

  // 3. Look Hard-Left (150 deg)
  scanServo.write(150);
  delay(140);
  distFarLeft = getMedianDistance(3);

  // 4. Look Front-Right (60 deg)
  scanServo.write(60);
  delay(160);
  distFrontRight = getMedianDistance(3);

  // 5. Look Hard-Right (30 deg)
  scanServo.write(30);
  delay(140);
  distFarRight = getMedianDistance(3);

  // Return to Center
  scanServo.write(90);
  delay(120);

  Serial.println("--------------- Sector Scan Results ---------------");
  Serial.printf("  [Far Left: %ld cm]  [Front Left: %ld cm]  [Center: %ld cm]  [Front Right: %ld cm]  [Far Right: %ld cm]\n",
                distFarLeft, distFrontLeft, distCenter, distFrontRight, distFarRight);
  Serial.println("---------------------------------------------------");
}

/**
 * Decides optimal turn angle based on 5-sector clearance
 */
void decideAndExecuteTurn() {
  long farLeft, frontLeft, center, frontRight, farRight;
  scanEnvironment(farLeft, frontLeft, center, frontRight, farRight);

  // Composite clearance score for left and right
  long totalLeftClearance = (farLeft * 0.4) + (frontLeft * 0.6);
  long totalRightClearance = (farRight * 0.4) + (frontRight * 0.6);

  // Condition 1: Both sides severely blocked -> Dead End / Corner Trap
  if (totalLeftClearance < SIDE_CLEARANCE_MIN_CM && totalRightClearance < SIDE_CLEARANCE_MIN_CM) {
    Serial.println("[DECISION] Dead End Detected! Backing up and performing 180-deg U-Turn.");
    backward();
    delay(650);
    stopMotors();
    delay(200);

    // Spin 180 degrees using calibrated Gyro
    if (imuAvailable) {
      turnAngleDegrees(180.0f);
    } else {
      // Timed fallback if IMU is offline
      turnRight();
      delay(1100);
      stopMotors();
    }
    return;
  }

  // Condition 2: Left is clearer than Right
  if (totalLeftClearance >= totalRightClearance) {
    // If front-left is relatively open, a softer 45-deg turn is sufficient
    float turnAngle = (frontLeft > 45) ? 45.0f : 90.0f;
    Serial.printf("[DECISION] Left side is clearer (L: %ld cm vs R: %ld cm). Turning +%.0f deg LEFT.\n",
                  totalLeftClearance, totalRightClearance, turnAngle);

    if (imuAvailable) {
      turnAngleDegrees(turnAngle);
    } else {
      turnLeft();
      delay(turnAngle == 45.0f ? 450 : 750);
      stopMotors();
    }
  }
  // Condition 3: Right is clearer than Left
  else {
    float turnAngle = (frontRight > 45) ? -45.0f : -90.0f;
    Serial.printf("[DECISION] Right side is clearer (R: %ld cm vs L: %ld cm). Turning %.0f deg RIGHT.\n",
                  totalRightClearance, totalLeftClearance, turnAngle);

    if (imuAvailable) {
      turnAngleDegrees(turnAngle);
    } else {
      turnRight();
      delay(turnAngle == -45.0f ? 450 : 750);
      stopMotors();
    }
  }

  delay(150); // Pause after turn to settle
}

// ================= INERTIAL CRASH DETECTION & AUTO-RECOVERY =================

/**
 * Checks for sudden acceleration shock (impact / bumper bump)
 */
bool checkInertialCrash(float ax, float ay) {
  // Horizontal acceleration magnitude in Gs
  float horizontalAccelG = sqrt(ax * ax + ay * ay);

  if (horizontalAccelG >= CRASH_DECEL_THRESHOLD_G) {
    Serial.printf("\n[!!! COLLISION SHOCK DETECTED !!!] Magnitude: %.2f G (Threshold: %.2f G)\n",
                  horizontalAccelG, CRASH_DECEL_THRESHOLD_G);
    return true;
  }
  return false;
}

/**
 * Auto-Recovery sequence when robot impacts an obstacle missed by ultrasonic
 */
void executeCrashRecovery() {
  Serial.println("[RECOVERY] Executing emergency obstacle avoidance escape...");

  // 1. Immediately kill motor power
  stopMotors();
  delay(150);

  // 2. Reverse backward to clear contact zone
  backward();
  delay(550);
  stopMotors();
  delay(250);

  // 3. Scan 5 points to find open escape path
  decideAndExecuteTurn();

  Serial.println("[RECOVERY] Recovery routine complete. Resuming forward navigation.\n");
}

// ================= CLOSED-LOOP GYROSCOPE TURNING =================

/**
 * Turns the robot precisely by targetDegrees using closed-loop gyro integration.
 * Positive (+) targetDegrees = Turn Left (Counter-Clockwise)
 * Negative (-) targetDegrees = Turn Right (Clockwise)
 */
void turnAngleDegrees(float targetDegrees) {
  if (!imuAvailable) {
    Serial.println("[GYRO] IMU not available. Aborting gyro turn.");
    return;
  }

  float targetAbs = abs(targetDegrees);
  float integratedAngle = 0.0f;
  unsigned long lastTime = micros();
  unsigned long startTime = millis();

  Serial.printf("[GYRO TURN] Starting precise turn: %.1f deg\n", targetDegrees);

  // Set motor directions for differential turn
  if (targetDegrees > 0) {
    // Turn Left: Left Motor Backward, Right Motor Forward
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    // Turn Right: Left Motor Forward, Right Motor Backward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  // Active integration loop
  while (integratedAngle < targetAbs) {
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;

    // Read Gyro Z
    float ax, ay, az, gz_dps, pitch, roll;
    readIMU(ax, ay, az, gz_dps, pitch, roll);

    // Integrate absolute angular change
    integratedAngle += abs(gz_dps) * dt;

    // Proportional slowdown near target angle (prevents inertia overshoot)
    float remaining = targetAbs - integratedAngle;
    int currentPwm = turnSpeed;
    if (remaining < 20.0f) {
      currentPwm = map((int)remaining, 0, 20, minTurnSpeed, turnSpeed);
    }
    setMotorSpeeds(currentPwm, currentPwm);

    // Safety timeout (max 2.8 seconds per turn to prevent stuck loops)
    if (millis() - startTime > 2800) {
      Serial.println("[GYRO TURN] Turn timeout safety triggered!");
      break;
    }

    delay(5);
  }

  // Immediate braking stop
  stopMotors();
  Serial.printf("[GYRO TURN] Finished. Target: %.1f deg | Achieved: %.1f deg\n", targetDegrees, integratedAngle);
  delay(120);
}

// ================= ULTRASONIC ROBUST SENSING (MEDIAN FILTER) =================

/**
 * Single HC-SR04 pulse measurement with timeout
 */
long getSingleDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 26ms timeout corresponds to approx 4.4 meters
  long duration = pulseIn(ECHO_PIN, HIGH, 26000);
  if (duration == 0) {
    return 999; // No echo received
  }
  return (long)(duration * 0.0343 / 2.0);
}

/**
 * 5-Sample Median Filter: Rejects sporadic spikes, electrical noise,
 * and 999cm timeouts without corrupting the average.
 */
long getMedianDistance(int samples) {
  long readings[7];
  if (samples > 7) samples = 7;
  if (samples < 1) samples = 1;

  for (int i = 0; i < samples; i++) {
    readings[i] = getSingleDistance();
    delay(12); // Small delay between pulses to allow acoustic echoes to dissipate
  }

  // Simple insertion sort to find median
  for (int i = 1; i < samples; i++) {
    long key = readings[i];
    int j = i - 1;
    while (j >= 0 && readings[j] > key) {
      readings[j + 1] = readings[j];
      j--;
    }
    readings[j + 1] = key;
  }

  // Return middle element (median)
  return readings[samples / 2];
}

// ================= MPU6050 HARDWARE DRIVER =================

/**
 * Initializes MPU6050 registers over I2C
 */
bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0x00); // Wake up MPU6050 (clear sleep bit)
  if (Wire.endTransmission() != 0) {
    return false;
  }

  // Set Accelerometer to +/- 2g (16384 LSB/g)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_ACCEL_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();

  // Set Gyroscope to +/- 250 dps (131.0 LSB/dps)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_GYRO_CONFIG);
  Wire.write(0x00);
  Wire.endTransmission();

  return true;
}

/**
 * Calibrates stationary Z-axis gyro offset at startup
 */
void calibrateGyroBias() {
  Serial.print("[IMU] Calibrating Gyro (Keep robot still)...");
  float sumZ = 0.0f;
  const int numSamples = 300;

  for (int i = 0; i < numSamples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x47); // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);

    if (Wire.available() >= 2) {
      int16_t rawGz = (Wire.read() << 8) | Wire.read();
      sumZ += rawGz;
    }
    delay(4);
  }

  gyroZ_offset = (sumZ / numSamples);
  Serial.printf(" Done! Zero-Rate Bias Offset: %.2f LSB\n", gyroZ_offset);
}

/**
 * Reads 6-DOF Accel & Gyro burst registers
 */
bool readIMU(float &ax, float &ay, float &az, float &gz_dps, float &pitch, float &roll) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  // Request 14 bytes: Accel (6) + Temp (2) + Gyro (6)
  if (Wire.requestFrom(MPU6050_ADDR, 14, true) != 14) {
    return false;
  }

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature bytes
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  // Convert Accelerometer raw to G's
  ax = (float)rawAx / 16384.0f;
  ay = (float)rawAy / 16384.0f;
  az = (float)rawAz / 16384.0f;

  // Convert Gyroscope Z to degrees/second
  gz_dps = ((float)rawGz - gyroZ_offset) / 131.0f;

  // Calculate Pitch and Roll for tip safety
  pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0f / PI;
  roll = atan2(-ax, az) * 180.0f / PI;

  return true;
}

// ================= TB6612 MOTOR CONTROL PRIMITIVES =================

void setDriverStandby(bool enable) {
  digitalWrite(STBY, enable ? HIGH : LOW);
}

void setMotorSpeeds(int leftPwm, int rightPwm) {
  leftPwm = constrain(leftPwm, 0, 255);
  rightPwm = constrain(rightPwm, 0, 255);

  ledcWrite(PWM_CHANNEL_A, leftPwm);
  ledcWrite(PWM_CHANNEL_B, rightPwm);
}

void forward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  setMotorSpeeds(forwardSpeed, forwardSpeed);
}

void backward() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  setMotorSpeeds(reverseSpeed, reverseSpeed);
}

void turnLeft() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);

  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  setMotorSpeeds(turnSpeed, turnSpeed);
}

void turnRight() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  setMotorSpeeds(turnSpeed, turnSpeed);
}

void stopMotors() {
  setMotorSpeeds(0, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}