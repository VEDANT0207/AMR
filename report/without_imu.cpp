/**
 * ============================================================
 * ESP32-S3 CUSTOMER-AWARE AUTONOMOUS MOBILE ROBOT
 * ============================================================
 *
 * Hardware:
 *
 *  TB6612FNG Motor Driver
 *  HC-SR04 Ultrasonic Sensor
 *  Mini Servo
 *  MPU6050 IMU
 *  Android YOLOv8-Pose Navigation
 *
 * ============================================================
 *
 * ANDROID COMMAND PROTOCOLS:
 *
 * 1. CSV:
 *
 *    CMD,DIRECTION,PWM,STEER,STOP_FLAG,SEQ
 *
 *    Example:
 *    CMD,FORWARD,255,0,0,1074
 *
 * 2. Compact:
 *
 *    STEER:15 SPEED:0.50
 *
 * 3. Emergency stop:
 *
 *    STOP
 *
 * ============================================================
 */

// ============================================================
// LIBRARIES
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ============================================================
// TB6612FNG MOTOR DRIVER
// ============================================================

// Left Motor A
#define AIN1 14
#define AIN2 15
#define PWMA 16

// Right Motor B
#define BIN1 17
#define BIN2 18
#define PWMB 4

// Standby
#define STBY 12

// ============================================================
// PWM CONFIGURATION
// ============================================================

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

#define PWM_CHANNEL_A 0
#define PWM_CHANNEL_B 1

// Maximum motor PWM.
// Android can send 255, but robot will only use 120.
#define MAX_MOTOR_PWM 120

// ============================================================
// ULTRASONIC SENSOR
// ============================================================

#define TRIG_PIN 21
#define ECHO_PIN 47

#define OBSTACLE_DISTANCE_THRESHOLD_CM 20

// ============================================================
// MINI SERVO
// ============================================================

#define SERVO_PIN 13

Servo scanServo;

// Servo scanning range
#define SERVO_MIN_ANGLE 65
#define SERVO_MAX_ANGLE 115

// Degrees moved per step
#define SERVO_STEP 3

// Time between servo movements
#define SERVO_INTERVAL_MS 100

int servoAngle = 65;
int servoDirection = 1;

unsigned long lastServoMoveTime = 0;

// ============================================================
// MPU6050
// ============================================================

#define MPU6050_ADDR 0x68

// YOUR ACTUAL CONNECTION
#define PIN_SDA 1
#define PIN_SCL 2

#define MAX_TILT_ANGLE_DEG 25.0

// ============================================================
// WATCHDOG
// ============================================================

#define WATCHDOG_TIMEOUT_MS 500

unsigned long lastCommandTime = 0;
unsigned long lastSensorPollTime = 0;

// ============================================================
// SAFETY STATE
// ============================================================

bool isFailsafeActive = true;

bool isObstacleDetected = false;

bool isImuTiltFault = false;

// ============================================================
// ANDROID COMMAND STATE
// ============================================================

float targetSteering = 0.0;

float targetSpeedScale = 0.0;

bool emergencyStopRequested = false;

// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void stopMotors();

void setMotorSpeeds(
    int leftPwm,
    int rightPwm
);

bool parseSerialCommand(
    String cmd
);

long readUltrasonicDistance();

bool initMPU6050();

bool checkIMUSafety();

void executeMovementControl();

void updateServo();

// ============================================================
// SETUP
// ============================================================

void setup() {

    // ========================================================
    // 1. ANDROID USB SERIAL
    // ========================================================

    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        "AMR ESP32-S3 UNIFIED FIRMWARE"
    );

    Serial.println(
        "========================================"
    );

    Serial.println(
        "STATUS:AMR_UNIFIED_FIRMWARE_INITIALIZING"
    );

    // ========================================================
    // 2. TB6612FNG MOTOR GPIO
    // ========================================================

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);

    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    pinMode(STBY, OUTPUT);

    // Enable TB6612FNG
    digitalWrite(
        STBY,
        HIGH
    );

    // ========================================================
    // PWM
    // ========================================================

    ledcSetup(
        PWM_CHANNEL_A,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcSetup(
        PWM_CHANNEL_B,
        PWM_FREQ,
        PWM_RESOLUTION
    );

    ledcAttachPin(
        PWMA,
        PWM_CHANNEL_A
    );

    ledcAttachPin(
        PWMB,
        PWM_CHANNEL_B
    );

    Serial.println(
        "STATUS:MOTOR_DRIVER:TB6612FNG"
    );

    Serial.println(
        "STATUS:MOTOR_DRIVER:ONLINE"
    );

    Serial.print(
        "STATUS:MAX_MOTOR_PWM:"
    );

    Serial.println(
        MAX_MOTOR_PWM
    );

    // ========================================================
    // 3. ULTRASONIC
    // ========================================================

    pinMode(
        TRIG_PIN,
        OUTPUT
    );

    pinMode(
        ECHO_PIN,
        INPUT
    );

    Serial.println(
        "STATUS:ULTRASONIC:ONLINE"
    );

    // ========================================================
    // 4. SERVO
    // ========================================================

    scanServo.attach(
        SERVO_PIN
    );

    servoAngle = SERVO_MIN_ANGLE;

    servoDirection = 1;

    scanServo.write(
        servoAngle
    );

    lastServoMoveTime = millis();

    Serial.println(
        "STATUS:SERVO:ONLINE"
    );

    Serial.println(
        "STATUS:SERVO:SCANNING"
    );

    Serial.print(
        "STATUS:SERVO:RANGE:"
    );

    Serial.print(
        SERVO_MIN_ANGLE
    );

    Serial.print(
        "-"
    );

    Serial.println(
        SERVO_MAX_ANGLE
    );

    // ========================================================
    // 5. MPU6050
    // ========================================================

    Serial.println(
        "STATUS:MPU6050:INITIALIZING"
    );

    Serial.print(
        "STATUS:MPU6050:SDA="
    );

    Serial.println(
        PIN_SDA
    );

    Serial.print(
        "STATUS:MPU6050:SCL="
    );

    Serial.println(
        PIN_SCL
    );

    Wire.begin(
        PIN_SDA,
        PIN_SCL,
        400000
    );

    bool imuOk =
        initMPU6050();

    if (imuOk) {

        Serial.println(
            "STATUS:MPU6050:ONLINE"
        );

    } else {

        Serial.println(
            "STATUS:MPU6050_NOT_DETECTED_CONTINUING"
        );
    }

    // ========================================================
    // 6. SAFE STARTUP
    // ========================================================

    stopMotors();

    lastCommandTime =
        millis();

    lastSensorPollTime =
        millis();

    isFailsafeActive =
        true;

    Serial.println(
        "STATUS:MOTORS:STOPPED"
    );

    Serial.println(
        "STATUS:WATCHDOG:500MS"
    );

    Serial.println(
        "STATUS:ANDROID_PROTOCOL:CSV_AND_COMPACT"
    );

    Serial.println(
        "STATUS:READY_FOR_ANDROID_COMMANDS"
    );

    Serial.println(
        "STATUS:USB_COMMANDS:READY"
    );

    Serial.println(
        "========================================"
    );
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

    unsigned long now =
        millis();

    // ========================================================
    // 1. READ ANDROID COMMAND STREAM
    // ========================================================

    while (
        Serial.available() > 0
    ) {

        String line =
            Serial.readStringUntil(
                '\n'
            );

        line.trim();

        if (
            line.length() > 0
        ) {

            bool valid =
                parseSerialCommand(
                    line
                );

            if (valid) {

                lastCommandTime =
                    now;

                isFailsafeActive =
                    false;

            } else {

                Serial.println(
                    "ERROR:INVALID_COMMAND"
                );
            }
        }
    }

    // ========================================================
    // 2. SERVO SCANNING
    // ========================================================

    updateServo();

    // ========================================================
    // 3. SENSOR POLLING
    // ========================================================

    if (
        now - lastSensorPollTime >= 50
    ) {

        lastSensorPollTime =
            now;

        // ----------------------------------------------------
        // Ultrasonic
        // ----------------------------------------------------

        long distance =
            readUltrasonicDistance();

        if (
            distance > 0 &&
            distance <
                OBSTACLE_DISTANCE_THRESHOLD_CM
        ) {

            isObstacleDetected =
                true;

        } else {

            isObstacleDetected =
                false;
        }

        // ----------------------------------------------------
        // MPU6050
        // ----------------------------------------------------

        isImuTiltFault =
            !checkIMUSafety();
    }

    // ========================================================
    // 4. COMMUNICATION WATCHDOG
    // ========================================================

    if (
        !isFailsafeActive &&
        (
            now - lastCommandTime >
            WATCHDOG_TIMEOUT_MS
        )
    ) {

        isFailsafeActive =
            true;

        Serial.println(
            "STATUS:WATCHDOG_TRIGGERED_MOTORS_HALTED"
        );
    }

    // ========================================================
    // 5. SAFETY + MOVEMENT CONTROL
    // ========================================================

    executeMovementControl();
}

// ============================================================
// SERVO SCANNING
// ============================================================

void updateServo() {

    unsigned long now =
        millis();

    if (
        now - lastServoMoveTime <
        SERVO_INTERVAL_MS
    ) {

        return;
    }

    lastServoMoveTime =
        now;

    // Move servo
    servoAngle +=
        servoDirection *
        SERVO_STEP;

    // --------------------------------------------------------
    // Hit maximum
    // --------------------------------------------------------

    if (
        servoAngle >=
        SERVO_MAX_ANGLE
    ) {

        servoAngle =
            SERVO_MAX_ANGLE;

        servoDirection =
            -1;
    }

    // --------------------------------------------------------
    // Hit minimum
    // --------------------------------------------------------

    if (
        servoAngle <=
        SERVO_MIN_ANGLE
    ) {

        servoAngle =
            SERVO_MIN_ANGLE;

        servoDirection =
            1;
    }

    scanServo.write(
        servoAngle
    );
}

// ============================================================
// MOVEMENT CONTROL
// ============================================================

void executeMovementControl() {

    // ========================================================
    // TIER 1
    // EMERGENCY STOP
    // ========================================================

    if (
        emergencyStopRequested
    ) {

        stopMotors();

        return;
    }

    // ========================================================
    // TIER 2
    // ULTRASONIC OBSTACLE
    // ========================================================

    if (
        isObstacleDetected
    ) {

        stopMotors();

        return;
    }

    // ========================================================
    // TIER 3
    // COMMUNICATION WATCHDOG
    // ========================================================

    if (
        isFailsafeActive
    ) {

        stopMotors();

        return;
    }

    // ========================================================
    // TIER 4
    // MPU6050 TILT
    // ========================================================

    if (
        isImuTiltFault
    ) {

        stopMotors();

        Serial.println(
            "SAFETY_TRIGGER:IMU_EXCESSIVE_TILT"
        );

        return;
    }

    // ========================================================
    // NO SPEED
    // ========================================================

    if (
        targetSpeedScale <= 0.01
    ) {

        stopMotors();

        return;
    }

    // ========================================================
    // DIFFERENTIAL DRIVE
    // ========================================================

    /*
     * IMPORTANT:
     *
     * Android PWM can be 0-255.
     *
     * We intentionally cap the actual motor PWM
     * using MAX_MOTOR_PWM.
     *
     * Example:
     *
     * Android PWM = 255
     *
     * targetSpeedScale = 1.0
     *
     * basePwm = 120
     *
     * Therefore the robot will not run at full speed.
     */

    float basePwm =
        targetSpeedScale *
        MAX_MOTOR_PWM;

    // Steering range approximately -45 to +45
    float turnFactor =
        (
            targetSteering /
            45.0
        ) * 0.5;

    // Limit turn factor
    turnFactor =
        constrain(
            turnFactor,
            -0.5,
            0.5
        );

    // --------------------------------------------------------
    // Differential drive
    // --------------------------------------------------------

    int leftPwm =
        (int)(
            basePwm *
            (
                1.0 -
                turnFactor
            )
        );

    int rightPwm =
        (int)(
            basePwm *
            (
                1.0 +
                turnFactor
            )
        );

    // Safety limit
    leftPwm =
        constrain(
            leftPwm,
            0,
            MAX_MOTOR_PWM
        );

    rightPwm =
        constrain(
            rightPwm,
            0,
            MAX_MOTOR_PWM
        );

    setMotorSpeeds(
        leftPwm,
        rightPwm
    );
}

// ============================================================
// ANDROID COMMAND PARSER
// ============================================================

bool parseSerialCommand(
    String cmd
) {

    // ========================================================
    // EMERGENCY STOP
    // ========================================================

    if (
        cmd == "STOP" ||
        cmd.startsWith("CMD,STOP")
    ) {

        emergencyStopRequested =
            true;

        targetSpeedScale =
            0.0;

        stopMotors();

        return true;
    }

    // ========================================================
    // COMPACT PROTOCOL
    //
    // STEER:15 SPEED:0.75
    // ========================================================

    if (
        cmd.startsWith("STEER:")
    ) {

        int speedIndex =
            cmd.indexOf(
                "SPEED:"
            );

        if (
            speedIndex != -1
        ) {

            targetSteering =
                cmd.substring(
                    6,
                    speedIndex
                ).toFloat();

            targetSpeedScale =
                cmd.substring(
                    speedIndex + 6
                ).toFloat();

            // Clamp speed
            targetSpeedScale =
                constrain(
                    targetSpeedScale,
                    0.0f,
                    1.0f
                );

            emergencyStopRequested =
                false;

            return true;
        }

        return false;
    }

    // ========================================================
    // CSV PROTOCOL
    //
    // CMD,DIRECTION,PWM,STEER,STOP_FLAG,SEQ
    //
    // Example:
    //
    // CMD,FORWARD,255,0,0,1074
    // ========================================================

    else if (
        cmd.startsWith("CMD,")
    ) {

        int firstComma =
            cmd.indexOf(',');

        int secondComma =
            cmd.indexOf(
                ',',
                firstComma + 1
            );

        int thirdComma =
            cmd.indexOf(
                ',',
                secondComma + 1
            );

        int fourthComma =
            cmd.indexOf(
                ',',
                thirdComma + 1
            );

        int fifthComma =
            cmd.indexOf(
                ',',
                fourthComma + 1
            );

        if (
            firstComma != -1 &&
            secondComma != -1 &&
            thirdComma != -1 &&
            fourthComma != -1
        ) {

            // ------------------------------------------------
            // PWM
            // ------------------------------------------------

            int pwmVal =
                cmd.substring(
                    secondComma + 1,
                    thirdComma
                ).toInt();

            // ------------------------------------------------
            // STEERING
            // ------------------------------------------------

            float steer =
                cmd.substring(
                    thirdComma + 1,
                    fourthComma
                ).toFloat();

            // ------------------------------------------------
            // STOP FLAG
            // ------------------------------------------------

            int stopFlag =
                cmd.substring(
                    fourthComma + 1,
                    (
                        fifthComma != -1
                        ?
                        fifthComma
                        :
                        cmd.length()
                    )
                ).toInt();

            // ------------------------------------------------
            // STOP REQUEST
            // ------------------------------------------------

            if (
                stopFlag == 1
            ) {

                emergencyStopRequested =
                    true;

                targetSpeedScale =
                    0.0;

                stopMotors();

                return true;
            }

            // ------------------------------------------------
            // NORMAL COMMAND
            // ------------------------------------------------

            emergencyStopRequested =
                false;

            targetSteering =
                steer;

            targetSpeedScale =
                constrain(
                    (float)pwmVal /
                    255.0f,
                    0.0f,
                    1.0f
                );

            return true;
        }

        return false;
    }

    return false;
}

// ============================================================
// HC-SR04 ULTRASONIC
// ============================================================

long readUltrasonicDistance() {

    digitalWrite(
        TRIG_PIN,
        LOW
    );

    delayMicroseconds(5);

    digitalWrite(
        TRIG_PIN,
        HIGH
    );

    delayMicroseconds(10);

    digitalWrite(
        TRIG_PIN,
        LOW
    );

    long duration =
        pulseIn(
            ECHO_PIN,
            HIGH,
            30000
        );

    // No echo
    if (
        duration == 0
    ) {

        return 999;
    }

    return (
        long
    )(
        duration *
        0.034 /
        2.0
    );
}

// ============================================================
// MPU6050 INITIALIZATION
// ============================================================

bool initMPU6050() {

    Wire.beginTransmission(
        MPU6050_ADDR
    );

    // Power management register
    Wire.write(0x6B);

    // Wake up
    Wire.write(0x00);

    byte error =
        Wire.endTransmission();

    return (
        error == 0
    );
}

// ============================================================
// MPU6050 SAFETY CHECK
// ============================================================

bool checkIMUSafety() {

    // Select accelerometer registers
    Wire.beginTransmission(
        MPU6050_ADDR
    );

    Wire.write(
        0x3B
    );

    if (
        Wire.endTransmission(false)
        != 0
    ) {

        // If sensor is disconnected,
        // don't falsely trigger tilt.
        return true;
    }

    // Explicit types
    // Avoid requestFrom warning
    uint8_t address =
        (uint8_t)MPU6050_ADDR;

    size_t bytesRequested =
        6;

    size_t received =
        Wire.requestFrom(
            address,
            bytesRequested,
            true
        );

    if (
        received == 6
    ) {

        // ----------------------------------------------------
        // RAW ACCELEROMETER
        // ----------------------------------------------------

        int16_t rawX =
            (Wire.read() << 8) |
            Wire.read();

        int16_t rawY =
            (Wire.read() << 8) |
            Wire.read();

        int16_t rawZ =
            (Wire.read() << 8) |
            Wire.read();

        // ----------------------------------------------------
        // CONVERT TO G
        // ±2g
        // ----------------------------------------------------

        float ax =
            rawX / 16384.0;

        float ay =
            rawY / 16384.0;

        float az =
            rawZ / 16384.0;

        // ----------------------------------------------------
        // PITCH
        // ----------------------------------------------------

        float pitch =
            atan2(
                ay,
                sqrt(
                    ax * ax +
                    az * az
                )
            )
            *
            180.0 /
            PI;

        // ----------------------------------------------------
        // ROLL
        // ----------------------------------------------------

        float roll =
            atan2(
                -ax,
                az
            )
            *
            180.0 /
            PI;

        // ----------------------------------------------------
        // TILT SAFETY
        // ----------------------------------------------------

        if (
            abs(pitch) >
                MAX_TILT_ANGLE_DEG
            ||
            abs(roll) >
                MAX_TILT_ANGLE_DEG
        ) {

            return false;
        }
    }

    return true;
}

// ============================================================
// MOTOR OUTPUT
// ============================================================

void setMotorSpeeds(
    int leftPwm,
    int rightPwm
) {

    // --------------------------------------------------------
    // SAFETY CLAMP
    // --------------------------------------------------------

    leftPwm =
        constrain(
            leftPwm,
            0,
            MAX_MOTOR_PWM
        );

    rightPwm =
        constrain(
            rightPwm,
            0,
            MAX_MOTOR_PWM
        );

    // --------------------------------------------------------
    // FORWARD MOTOR POLARITY
    // --------------------------------------------------------

    digitalWrite(
        AIN1,
        HIGH
    );

    digitalWrite(
        AIN2,
        LOW
    );

    digitalWrite(
        BIN1,
        HIGH
    );

    digitalWrite(
        BIN2,
        LOW
    );

    // --------------------------------------------------------
    // PWM
    // --------------------------------------------------------

    ledcWrite(
        PWM_CHANNEL_A,
        leftPwm
    );

    ledcWrite(
        PWM_CHANNEL_B,
        rightPwm
    );
}

// ============================================================
// IMMEDIATE MOTOR STOP
// ============================================================

void stopMotors() {

    ledcWrite(
        PWM_CHANNEL_A,
        0
    );

    ledcWrite(
        PWM_CHANNEL_B,
        0
    );

    digitalWrite(
        AIN1,
        LOW
    );

    digitalWrite(
        AIN2,
        LOW
    );

    digitalWrite(
        BIN1,
        LOW
    );

    digitalWrite(
        BIN2,
        LOW
    );
}