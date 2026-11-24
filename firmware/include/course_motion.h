#pragma once
#include <Wire.h>
#include "zumo.h"

// Steering and gyro routines used by the course behaviors in main.cpp.
const unsigned int SIM_LINE_SPAN_US[6] = {3038, 2250, 1800, 2363, 2475, 2700};
constexpr uint8_t IMU_ADDRESS = 0x6B;
constexpr int LANE_SPEED = 200, CURVE_SPEED = 180, LINE_SPEED = 250;
// The bend begins about 44 cm from the nominal start. Bound straight travel if the IR misses its edge.
constexpr float MAX_FIRST_STRAIGHT_CM = 47.0f;
constexpr float CURVE_RADIUS_X_CM = 40.0f, CURVE_RADIUS_Y_CM = 52.0f;
constexpr float TRACK_WIDTH_CM = 8.5f, MOTOR_DEADBAND = 48.0f;
constexpr float MODEL_MAX_CM_S = 52.0f, MODEL_RIGHT_GAIN = 0.97f;
constexpr float MODEL_MOTOR_TAU_S = 0.07f;
#if TP1_SIMULATION
constexpr float RIGHT_TRIM = 1.031f;
#else
constexpr float RIGHT_TRIM = 0.990f;
#endif
constexpr float LANE_KP = 0.095f, LANE_KD = 0.0007f;
constexpr float LINE_KP = 28.0f, LINE_KD = 0.35f;
constexpr float LINE_HEADING_KP = 2.0f, LINE_YAW_KD = 0.20f;
constexpr float LINE_FILTER_S = 0.025f;

unsigned long lastGyroUs = 0, lineLastSeen = 0, conditionStart = 0;
float heading = 0, yawRate = 0, gyroBias = 0, biasSum = 0;
float targetHeading = 0, previousError = 0, filteredDerivative = 0;
float filteredLineError = 0;
float lastLinePosition = 0;
bool lineFilterReady = false;
uint16_t biasCount = 0;
uint8_t lanePhase = 0;
bool firstCurveApexSeen = false;
bool exitLeftEdgeAcquired = false;
unsigned long exitEdgeSeenStart = 0;
unsigned long exitEdgeSearchStart = 0;
bool thirdSensorOnEdge = false;
float firstStraightCm = 0;
float estimatedLeftSpeed = 0, estimatedRightSpeed = 0;
int commandedLeft = 0, commandedRight = 0;

float limitFloat(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void drive(int left, int right) {
    commandedLeft = constrain(left, -MOTOR_TOP, MOTOR_TOP);
    commandedRight = constrain((int)(right * RIGHT_TRIM), -MOTOR_TOP, MOTOR_TOP);
    setSpeeds(commandedLeft, commandedRight);
}

void resetSteering() {
    conditionStart = 0;
    previousError = 0;
    filteredDerivative = 0;
    lineFilterReady = false;
    lastLinePosition = 0;
    lineLastSeen = millis();
}

bool held(bool condition, unsigned long duration) {
    if (!condition) { conditionStart = 0; return false; }
    if (conditionStart == 0) conditionStart = millis();
    return millis() - conditionStart >= duration;
}

bool readRegisters(uint8_t reg, uint8_t count) {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    return Wire.requestFrom(IMU_ADDRESS, count) == count;
}

bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IMU_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool beginGyro() {
    Wire.begin();
    Wire.setClock(400000);
    Wire.setWireTimeout(3000, true);
    if (!readRegisters(0x0F, 1) || Wire.read() != 0x69) return false;
    // LSM6DS33: auto-increment, BDU; 208 Hz, +/-1000 dps (35 mdps/LSB).
    return writeRegister(0x12, 0x44) && writeRegister(0x11, 0x58);
}

bool updateGyro(bool waiting, unsigned long stateMs) {
    if (!readRegisters(0x26, 2)) return false;
    uint8_t lo = Wire.read();
    int16_t raw = (int16_t)((uint16_t)Wire.read() << 8 | lo);
    float rate = raw * 0.035f;
    unsigned long now = micros();
    float dt = (now - lastGyroUs) * 0.000001f;
    lastGyroUs = now;
    if (waiting) {
        if (stateMs > 300 && biasCount < 200) {
            biasSum += rate;
            ++biasCount;
            gyroBias = biasSum / biasCount;
            if (biasCount == 200) {
                digitalWrite(LED_PIN, HIGH);  // yellow LED on: gyro calibrated
            }
        }
        heading = 0;
    } else if (dt < 0.1f) heading += (rate - gyroBias) * dt;
    yawRate = rate - gyroBias;
    return true;
}

int darkCount(int threshold) {
    int count = 0;
    for (uint8_t i = 0; i < 6; ++i) if (irValue[i] > threshold) ++count;
    return count;
}

float derivative(float current, float dt) {
    float d = (current - previousError) / dt;
    previousError = current;
    filteredDerivative += 0.25f * (d - filteredDerivative);
    return filteredDerivative;
}

void headingDrive(int base, float desired) {
    float c = limitFloat((heading - desired) * 3.2f + yawRate * 0.16f, -90, 90);
    drive(base + c, base - c);
}

float modelSpeed(int command, float gain) {
    float magnitude = max(0.0f, abs(command) - MOTOR_DEADBAND);
    float speed = gain * MODEL_MAX_CM_S * magnitude / (MOTOR_TOP - MOTOR_DEADBAND);
    return command < 0 ? -speed : speed;
}

float forwardDistance(float dt) {
    // Estimate displacement from motor commands and the gyro.
    float blend = dt / (MODEL_MOTOR_TAU_S + dt);
    estimatedLeftSpeed += blend * (modelSpeed(commandedLeft, 1.0f) - estimatedLeftSpeed);
    estimatedRightSpeed += blend * (modelSpeed(commandedRight, MODEL_RIGHT_GAIN) - estimatedRightSpeed);
    float forward = (estimatedLeftSpeed + estimatedRightSpeed) * 0.5f;
    return forward * dt * cosf((heading - targetHeading) * DEG_TO_RAD);
}

float curveFeedForward() {
    // Curvature of the course's elliptical center path, from gyro heading.
    // No simulator position or scoring information is used by the firmware.
    float a2 = CURVE_RADIUS_X_CM * CURVE_RADIUS_X_CM;
    float b2 = CURVE_RADIUS_Y_CM * CURVE_RADIUS_Y_CM;
    float cosine = cosf(heading * DEG_TO_RAD);
    float q = a2 + (b2 - a2) * cosine * cosine;
    float curvature = q * sqrtf(q) / (a2 * b2);
    float magnitude = (CURVE_SPEED - MOTOR_DEADBAND) * TRACK_WIDTH_CM * curvature * 0.5f;
    return lanePhase == 1 ? -magnitude : magnitude;
}

void followLaneEdges(float dt) {
    int left = max(irValue[0], max(irValue[1], irValue[2]));
    int right = max(irValue[3], max(irValue[4], irValue[5]));
    if (lanePhase == 0 && (right > 100 || firstStraightCm >= MAX_FIRST_STRAIGHT_CM)) {
        lanePhase = 1;
        firstCurveApexSeen = false;
    }
    if (lanePhase == 1) {
        if (heading > 150) firstCurveApexSeen = true;
        if (heading > 165 || (firstCurveApexSeen && heading < 145)) lanePhase = 2;
    }
    if (lanePhase == 2 && heading < 7) {
        lanePhase = 3;
        exitLeftEdgeAcquired = false;
        exitEdgeSeenStart = 0;
        exitEdgeSearchStart = millis();
    }
    float error = left - right;
    if (lanePhase == 0) headingDrive(LANE_SPEED, 0);
    else if (lanePhase == 3) {
        int leftEdge = max(irValue[0], irValue[1]);
        int leftThree = max(leftEdge, (int)irValue[2]);
        thirdSensorOnEdge = irValue[2] >= 100;
        if (!exitLeftEdgeAcquired) {
            if (leftThree >= 100) {
                if (exitEdgeSeenStart == 0) exitEdgeSeenStart = millis();
                if (millis() - exitEdgeSeenStart >= 30) exitLeftEdgeAcquired = true;
            } else exitEdgeSeenStart = 0;
            if (millis() - exitEdgeSearchStart >= 3000) exitLeftEdgeAcquired = true;
            if (!exitLeftEdgeAcquired) {
                headingDrive(125, 15.0f);
                return;
            }
        }
        float desiredHeading = limitFloat((200 - leftEdge) * 0.045f, -12.0f, 8.0f);
        if (thirdSensorOnEdge) desiredHeading = -12.0f;
        headingDrive(160, desiredHeading);
    }
    else {
        float feedForward = curveFeedForward();
        // Bend one was consistently about 1 cm too far toward its inner edge
        // in repeated simulation runs. A small outward bias only affects it.
        float firstCurveOutwardBias = lanePhase == 1 ? 6.0f : 0.0f;
        float c = feedForward + firstCurveOutwardBias +
                  LANE_KP * error + LANE_KD * derivative(error, dt);
        c = limitFloat(c, -90, 90);
        int base = CURVE_SPEED - min(25, max(0, abs((int)c) - 30) / 3);
        drive(base + c, base - c);
    }
}

bool readLinePosition(bool simulation, float &position) {
    long total = 0, weighted = 0;
    const int8_t weights[6] = {-5, -3, -1, 1, 3, 5};
    for (uint8_t i = 0; i < 6; ++i) {
        int signal = simulation ?
            ((long)irRaw[i] - irWhite[i]) * 1000 / SIM_LINE_SPAN_US[i] : irValue[i];
        // Equalize partial-black responses and cap the central pair before clipping.
        int v = max(0, constrain(signal, 0, 700) - 50);
        total += v;
        weighted += (long)weights[i] * v;
    }
    if (total < 130) return false;
    position = (float)weighted / total;
    return true;
}

void steerOnLine(float dt, float measuredError, int baseSpeed) {
    lineLastSeen = millis();
    lastLinePosition = measuredError;
    if (!lineFilterReady) {
        filteredLineError = measuredError;
        previousError = measuredError;
        filteredDerivative = 0;
        lineFilterReady = true;
    } else filteredLineError += dt / (LINE_FILTER_S + dt) * (measuredError - filteredLineError);
    float c = LINE_KP * filteredLineError + LINE_KD * derivative(filteredLineError, dt) +
        LINE_HEADING_KP * (heading - targetHeading) + LINE_YAW_KD * yawRate;
    c = limitFloat(c, -120, 120);
    int base = baseSpeed - min(65, (int)(abs(filteredLineError) * 18));
    drive(base + c, base - c);
}

void trackLine(float dt, bool simulation, int baseSpeed) {
    float position;
    if (readLinePosition(simulation, position)) steerOnLine(dt, position, baseSpeed);
    else {
        lineFilterReady = false;
        uint8_t strongestIndex = 0;
        int strongestValue = irValue[0];
        for (uint8_t i = 1; i < 6; ++i) {
            if (irValue[i] > strongestValue) {
                strongestValue = irValue[i];
                strongestIndex = i;
            }
        }
        if (strongestValue >= 50) {
            int direction = strongestIndex >= 3 ? 1 : -1;
            int searchTurn = (strongestIndex == 0 || strongestIndex == 5) ? 55 : 40;
            drive(115 + direction * searchTurn, 115 - direction * searchTurn);
        } else if (abs(lastLinePosition) < 0.2f) headingDrive(150, targetHeading);
        else {
            int searchTurn = constrain(35 + (int)(abs(lastLinePosition) * 12), 35, 65);
            int direction = lastLinePosition > 0 ? 1 : -1;
            drive(120 + direction * searchTurn, 120 - direction * searchTurn);
        }
    }
}

bool rotateTo(float angle) {
    float remaining = angle - heading;
    int speed = constrain((int)(abs(remaining) * 1.5f), 65, 125);
    if (abs(remaining) < 2.5f) { drive(0, 0); return held(true, 100); }
    conditionStart = 0;
    if (remaining > 0) drive(-speed, speed);
    else drive(speed, -speed);
    return false;
}
