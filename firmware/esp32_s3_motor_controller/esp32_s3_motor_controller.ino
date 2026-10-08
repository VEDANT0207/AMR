/**
 * ESP32-S3 Unified Customer-Aware Autonomous Mobile Robot (AMR) Firmware
 * 
 * Hardware Subsystems (Verified from hardware/ sketches):
 *  1. TB6612FNG Dual H-Bridge Motor Driver (PWMA=16, PWMB=4, AIN1=14, AIN2=15, BIN1=17, BIN2=18, STBY=12)
 *  2. HC-SR04 Ultrasonic Distance Sensor (TRIG=21, ECHO=47) on Servo (SERVO=13)
 *  3. MPU6050 6-DOF IMU (I2C SDA=8, SCL=9, Addr=0x68) for Tilt & Rollover Safety
 *  4. High-Speed USB-CDC / UART Serial (115200 Baud) Command Receiver from Android YOLOv8-Pose
 *  5. 500ms Hardware Communication Watchdog Failsafe
 * 
 * SAFETY ARBITRATION PRIORITY MATRIX:
 *  [Tier 1] EMERGENCY STOP (Android UI / STOP Command Override)
 *  [Tier 2] Ultrasonic Obstacle Safety (Front Distance < 20cm Proximity Bumper)
 *  [Tier 3] Communication Watchdog (Timeout > 500ms -> Immediate Motor Halt)
 *  [Tier 4] MPU6050 Tilt / Impact Failsafe (Pitch / Roll > 25.0 deg)
 *  [Tier 5] Android YOLO Navigation Commands (Differential Drive STEER & SPEED)
 */

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ================= TB6612FNG MOTOR DRIVER PIN CONFIGURATION =================
// Left Motor (Motor A)
#define AIN1 14
#define AIN2 15
#define PWMA 16

// Right Motor (Motor B)
#define BIN1 17
#define BIN2 18
#define PWMB 4

// Standby Pin (Must be pulled HIGH to enable TB6612FNG bridge)
#define STBY 12

// ================= PWM CHANNELS & FREQUENCY =================
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

// ================= ULTRASONIC SENSOR & SERVO =================
#define TRIG_PIN 21
#define ECHO_PIN 47
#define SERVO_PIN 13
#define OBSTACLE_DISTANCE_THRESHOLD_CM 20

Servo scanServo;

// ================= MPU6050 IMU CONFIGURATION =================
#define MPU6050_ADDR 0x68
#define PIN_SDA 8
#define PIN_SCL 9
#define MAX_TILT_ANGLE_DEG 25.0

// ================= WATCHDOG & TIMING =================
#define WATCHDOG_TIMEOUT_MS 500

unsigned long lastCommandTime = 0;
unsigned long lastSensorPollTime = 0;
bool isFailsafeActive = true;
bool isObstacleDetected = false;
bool isImuTiltFault = false;

// Commanded values from Android YOLO Navigation
float targetSteering = 0.0;
float targetSpeedScale = 0.0;
bool emergencyStopRequested = false;

// ================= FUNCTION DECLARATIONS =================
void stopMotors();
void setMotorSpeeds(int leftPwm, int rightPwm);
bool parseSerialCommand(String cmd);
long readUltrasonicDistance();
bool initMPU6050();
bool checkIMUSafety();
void executeMovementControl();

// ================= SETUP =================
void setup() {
  // 1. Initialize Serial for Android USB-C OTG Communication
  Serial.begin(115200);
  delay(1000);
  Serial.println("STATUS:AMR_UNIFIED_FIRMWARE_INITIALIZING");

  // 2. Setup TB6612FNG Motor GPIOs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable TB6612FNG Motor Driver
  digitalWrite(STBY, HIGH);

  // Setup PWM Channels
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL_A);
  ledcAttachPin(PWMB, PWM_CHANNEL_B);

  Serial.println("STATUS:MOTOR_DRIVER:TB6612FNG");

  // 3. Setup Ultrasonic & Servo Subsystems
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  scanServo.attach(SERVO_PIN);
  scanServo.write(90); // Initial Center forward position
  Serial.println("STATUS:ULTRASONIC:ONLINE");
  Serial.println("STATUS:SERVO:ONLINE");

  // 4. Setup I2C & MPU6050 IMU
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  bool imuOk = initMPU6050();
  if (imuOk) {
    Serial.println("STATUS:MPU6050:ONLINE");
  } else {
    Serial.println("STATUS:MPU6050_NOT_DETECTED_CONTINUING");
  }

  // 5. Safe Startup State
  stopMotors();
  lastCommandTime = millis();
  isFailsafeActive = true;

  Serial.println("STATUS:READY_FOR_ANDROID_COMMANDS");
  Serial.println("STATUS:USB_COMMANDS:READY");
}

// ================= MAIN LOOP =================
void loop() {
  unsigned long now = millis();

  // 1. Read Serial Command Stream from Android Phone
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      bool valid = parseSerialCommand(line);
      if (valid) {
        lastCommandTime = now;
        isFailsafeActive = false;
      } else {
        Serial.println("ERROR:INVALID_COMMAND");
      }
    }
  }

  // 2. Periodic Sensor Polling (every 50ms)
  if (now - lastSensorPollTime >= 50) {
    lastSensorPollTime = now;

    // Check Ultrasonic Proximity
    long distance = readUltrasonicDistance();
    if (distance > 0 && distance < OBSTACLE_DISTANCE_THRESHOLD_CM) {
      isObstacleDetected = true;
    } else {
      isObstacleDetected = false;
    }

    // Check IMU Tilt
    isImuTiltFault = !checkIMUSafety();
  }

  // 3. Check Communication Watchdog (500ms timeout)
  if (!isFailsafeActive && (now - lastCommandTime > WATCHDOG_TIMEOUT_MS)) {
    isFailsafeActive = true;
    Serial.println("STATUS:WATCHDOG_TRIGGERED_MOTORS_HALTED");
  }

  // 4. Multi-Tier Movement Arbitration
  executeMovementControl();
}

/**
 * Multi-Tier Safety Execution Arbiter
 * Guarantees Android commands CANNOT bypass hardware safety triggers.
 */
void executeMovementControl() {
  // Tier 1: Emergency Stop Command from Android UI
  if (emergencyStopRequested) {
    stopMotors();
    return;
  }

  // Tier 2: Ultrasonic Obstacle Safety (Hardware Proximity Bumper)
  if (isObstacleDetected) {
    stopMotors();
    Serial.println("SAFETY_TRIGGER:ULTRASONIC_CLOSE_OBSTACLE");
    return;
  }

  // Tier 3: Communication Watchdog Failsafe
  if (isFailsafeActive) {
    stopMotors();
    return;
  }

  // Tier 4: IMU Tilt / Tip Fault
  if (isImuTiltFault) {
    stopMotors();
    Serial.println("SAFETY_TRIGGER:IMU_EXCESSIVE_TILT");
    return;
  }

  // Tier 5: Execute YOLO Navigation Movement Command
  if (targetSpeedScale <= 0.01) {
    stopMotors();
    return;
  }

  // Compute Differential Drive PWM
  // Positive (+) steering = Steer Left (Counter-Clockwise) -> Left Motor slower, Right Motor faster
  // Negative (-) steering = Steer Right (Clockwise) -> Left Motor faster, Right Motor slower
  float basePwm = targetSpeedScale * 255.0;
  float turnFactor = (targetSteering / 45.0) * 0.5; // [-0.5 .. +0.5]

  int leftPwm = (int)(basePwm * (1.0 - turnFactor));
  int rightPwm = (int)(basePwm * (1.0 + turnFactor));

  setMotorSpeeds(leftPwm, rightPwm);
}

/**
 * Parses both CSV ("CMD,DIR,PWM,STEER,STOP_FLAG,SEQ") 
 * and Compact ("STEER:X SPEED:Y" / "STOP") protocols.
 * Returns true if the command is valid.
 */
bool parseSerialCommand(String cmd) {
  // Emergency stop check
  if (cmd == "STOP" || cmd.startsWith("CMD,STOP")) {
    emergencyStopRequested = true;
    targetSpeedScale = 0.0;
    stopMotors();
    return true;
  }

  if (cmd.startsWith("STEER:")) {
    // Compact format: STEER:15 SPEED:0.75
    int speedIndex = cmd.indexOf("SPEED:");
    if (speedIndex != -1) {
      targetSteering = cmd.substring(6, speedIndex).toFloat();
      targetSpeedScale = cmd.substring(speedIndex + 6).toFloat();
      emergencyStopRequested = false;
      return true;
    }
    return false;
  } else if (cmd.startsWith("CMD,")) {
    // CSV format: CMD,DIRECTION,PWM,STEER,STOP_FLAG,SEQ
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    int thirdComma = cmd.indexOf(',', secondComma + 1);
    int fourthComma = cmd.indexOf(',', thirdComma + 1);
    int fifthComma = cmd.indexOf(',', fourthComma + 1);

    if (firstComma != -1 && secondComma != -1 && thirdComma != -1 && fourthComma != -1) {
      int pwmVal = cmd.substring(secondComma + 1, thirdComma).toInt();
      float steer = cmd.substring(thirdComma + 1, fourthComma).toFloat();
      int stopFlag = cmd.substring(fourthComma + 1, fifthComma != -1 ? fifthComma : cmd.length()).toInt();

      if (stopFlag == 1) {
        emergencyStopRequested = true;
        targetSpeedScale = 0.0;
        stopMotors();
        return true;
      }

      emergencyStopRequested = false;
      targetSteering = steer;
      targetSpeedScale = constrain((float)pwmVal / 255.0f, 0.0f, 1.0f);
      return true;
    }
    return false;
  }

  return false;
}

/**
 * Reads HC-SR04 Ultrasonic Distance in cm
 */
long readUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5 meters)
  if (duration == 0) {
    return 999;
  }
  return (long)(duration * 0.034 / 2.0);
}

/**
 * Initializes MPU6050 over I2C
 */
bool initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // Power management register
  Wire.write(0x00); // Wake up MPU6050
  byte error = Wire.endTransmission();
  return (error == 0);
}

/**
 * Checks MPU6050 accelerometer tilt angles
 */
bool checkIMUSafety() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Accel data registers
  if (Wire.endTransmission(false) != 0) {
    return true; // Non-fatal if IMU disconnected
  }

  if (Wire.requestFrom(MPU6050_ADDR, 6, true) == 6) {
    int16_t rawX = (Wire.read() << 8) | Wire.read();
    int16_t rawY = (Wire.read() << 8) | Wire.read();
    int16_t rawZ = (Wire.read() << 8) | Wire.read();

    float ax = rawX / 16384.0;
    float ay = rawY / 16384.0;
    float az = rawZ / 16384.0;

    float pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
    float roll = atan2(-ax, az) * 180.0 / PI;

    if (abs(pitch) > MAX_TILT_ANGLE_DEG || abs(roll) > MAX_TILT_ANGLE_DEG) {
      return false; // Excessive tilt detected!
    }
  }
  return true;
}

/**
 * Sets PWM duty cycle and direction for TB6612FNG driver
 */
void setMotorSpeeds(int leftPwm, int rightPwm) {
  leftPwm = constrain(leftPwm, 0, 255);
  rightPwm = constrain(rightPwm, 0, 255);

  // Forward Direction Polarity (matching hardware/ultrasonic-tfng-auto-test.cpp)
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  // Apply PWM speeds to TB6612FNG PWMA and PWMB
  ledcWrite(PWM_CHANNEL_A, leftPwm);
  ledcWrite(PWM_CHANNEL_B, rightPwm);
}

/**
 * Immediate Hardware Stop for TB6612FNG
 */
void stopMotors() {
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}
