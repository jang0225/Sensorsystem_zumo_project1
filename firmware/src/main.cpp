// Term Project 1: the seven states and behavior functions from the starter.
#ifndef TP1_SIMULATION
#define TP1_SIMULATION 0
#endif
#include <Arduino.h>
#include <EEPROM.h>
#include "zumo.h"
#include "course_motion.h"

enum State { WAIT, LANE, OBSTACLES, AVOID, CHECKPOINT, RETURN, FINISH };
State state = WAIT, resumeState = OBSTACLES;
unsigned long stateStart = 0;
constexpr unsigned long START_DELAY_MS = 1000;
bool startDelayActive = false;
unsigned long startDelayBegin = 0;

constexpr bool SIMULATION = TP1_SIMULATION;
constexpr bool REAL_CALIBRATION_READY = true;
constexpr bool USE_ULTRASONIC = true;  // Set false for a junction-only trial.
constexpr bool CALIBRATION_MODE = !SIMULATION && !REAL_CALIBRATION_READY;
const unsigned int REAL_WHITE[6] = {188, 156, 116, 156, 156, 156};
const unsigned int REAL_BLACK[6] = {2000, 2000, 2000, 2000, 2000, 2000};
const unsigned int SIM_WHITE[6] = {203, 150, 120, 158, 165, 180};
const unsigned int SIM_BLACK[6] = {2000, 2000, 1920, 2000, 2000, 2000};

enum LaneStep { BETWEEN_EDGES, CENTER_ON_JUNCTION, PAUSE_ON_JUNCTION, TURN_ONTO_LINE, FIND_ENTRY_LINE, ON_LINE };
enum PadStep { ENTER_PAD, STOP_ON_PAD, TURN_ON_PAD, LEAVE_PAD, OFF_PAD };
LaneStep laneStep = BETWEEN_EDGES;
PadStep padStep = ENTER_PAD;
unsigned long stepStart = 0, lastControl = 0, runStart = 0, returnDriveMs = 0;
float dt = 0, sectionTravelCm = 0, padTravelCm = 0;
float gateClearTravelCm = -100.0f;
float junctionTravelCm = 0;
constexpr float JUNCTION_ADVANCE_CM = 3.5f;
int entrySearchTurn = 80;  // With no IR sighting, look toward the overshoot side first.
constexpr float GATE_STOP_CM = 22.0f;
constexpr float GATE_CLEAR_CM = 27.0f;
constexpr float JUNCTION_TRIAL_STOP_CM = 37.0f;
constexpr float CHECKPOINT_APPROACH_CM = 136.0f;  // Beyond the second gate at 128 cm.
constexpr int GATE_CROSSING_SPEED = 190;  // Pace the 64 cm between the alternating gates.
constexpr int OUTBOUND_SPEED = 320;
constexpr int RETURN_SPEED = 320;
bool gateWaitCompleted = false;  // One wait per direction, reset after the checkpoint U-turn.

void startStep() {
    drive(0, 0);
    resetSteering();
    stepStart = millis();
}

void saveTrace();
void dumpTrace();
void enterState(State next) {
    static const char *const NAME[] = {"WAIT", "LANE", "OBSTACLES", "AVOID", "CHECKPOINT", "RETURN", "FINISH"};
    state = next;
    startStep();
    stateStart = stepStart;
    Serial.print(F("state: "));
    Serial.println(NAME[next]);
    if (next == FINISH) saveTrace();
}

unsigned long timeInState() { return millis() - stateStart; }

void stopRun(const __FlashStringHelper *reason) {
    enterState(FINISH);
    Serial.println(reason);
}

// ---- Conditions: when to change state -------------------------------------
bool zoneReached() { return laneStep == ON_LINE; }

bool junctionDetected() {
    if (lanePhase != 3 || abs(heading) > 25.0f) return false;
    return held(darkCount(650) >= 5, 20);
}

bool obstacleClose() {
    if (!USE_ULTRASONIC) return state == OBSTACLES && sectionTravelCm >= JUNCTION_TRIAL_STOP_CM;
    return !gateWaitCompleted && usCm > 0.0f && usCm <= GATE_STOP_CM;
}

bool pathClear() {
    // The supplied sonar helper reports 999 for no echo, including open space.
    return held(usCm > GATE_CLEAR_CM, 120);
}

bool widePad() {
    return darkCount(430) >= 5 && irValue[0] > 350 && irValue[5] > 350;
}

bool onCheckpointPad() {
    return state == OBSTACLES && sectionTravelCm >= CHECKPOINT_APPROACH_CM && widePad();
}

bool checkpointReached() { return held(onCheckpointPad(), 80); }
bool uTurnDone() { return padStep == OFF_PAD; }

bool finishReached() {
    return state == RETURN && millis() - lineLastSeen > 350 &&
           sectionTravelCm >= 205.0f && returnDriveMs >= 7000 && abs(heading - targetHeading) < 10;
}

// ---- Behaviors ------------------------------------------------------------
bool entryLineCentered(float position) {
    return max(irValue[2], irValue[3]) > 250 && abs(position) <= 1.5f;
}

void findEntryLine() {
    static bool reversedSweep = false;
    if (millis() - stepStart >= 4500) {
        stopRun(F("Entry line not found after right turn"));
        return;
    }
    float position;
    if (darkCount(250) > 0 && readLinePosition(SIMULATION, position)) {
        bool centered = entryLineCentered(position);
        steerOnLine(dt, position, centered ? 130 : 100);
        if (held(centered && abs(heading - targetHeading) <= 5, 60)) laneStep = ON_LINE;
        return;
    }

    // All white: pivot toward the last sighting, then check the opposite side.
    conditionStart = 0;
    lineFilterReady = false;
    bool atLimit = entrySearchTurn > 0 ? heading <= targetHeading - 35 : heading >= targetHeading + 35;
    if (atLimit) {
        if (reversedSweep) { stopRun(F("Entry line not found after right turn")); return; }
        entrySearchTurn = -entrySearchTurn;
        reversedSweep = true;
    }
    drive(entrySearchTurn, -entrySearchTurn);
}

void keepLane() {
    if (state != LANE) {
        if (onCheckpointPad()) {
            headingDrive(110, targetHeading);
            return;
        }
        int lineSpeed = state == RETURN ? RETURN_SPEED :
                        state == OBSTACLES ? OUTBOUND_SPEED :
                        (gateWaitCompleted ? GATE_CROSSING_SPEED : LINE_SPEED);
        trackLine(dt, SIMULATION, lineSpeed);
        if (millis() - lineLastSeen > 350 && !finishReached()) stopRun(F("Center line lost"));
        return;
    }

    switch (laneStep) {
        case BETWEEN_EDGES:
            followLaneEdges(dt);
            if (junctionDetected()) {
                startStep();
                laneStep = CENTER_ON_JUNCTION;
            }
            break;
        case CENTER_ON_JUNCTION:
            // The IR array is ahead of the axle. Position the pivot over the tape first.
            if (junctionTravelCm >= JUNCTION_ADVANCE_CM) {
                startStep();
                laneStep = PAUSE_ON_JUNCTION;
            } else headingDrive(120, 0);
            break;
        case PAUSE_ON_JUNCTION:
            if (millis() - stepStart >= 150) {
                startStep();
                targetHeading = -90;
                laneStep = TURN_ONTO_LINE;
            }
            break;
        case TURN_ONTO_LINE: {
            float position;
            // Near the end of the pivot the center sensors can cross the tape before 90 degrees.
            bool centered = abs(heading - targetHeading) <= 14 &&
                            readLinePosition(SIMULATION, position) && entryLineCentered(position);
            if (abs(heading - targetHeading) < 25) {
                if (irValue[0] > 250 && irValue[0] > irValue[5]) entrySearchTurn = -80;
                else if (irValue[5] > 250) entrySearchTurn = 80;
            }
            if (centered || rotateTo(targetHeading)) {
                startStep();
                laneStep = FIND_ENTRY_LINE;
            }
            else if (millis() - stepStart > 4000) stopRun(F("Entry turn timeout"));
            break;
        }
        case FIND_ENTRY_LINE:
            findEntryLine();
            break;
        case ON_LINE:
            break;
    }
}

void goAround() {
    // The gate moves, so stay stopped until it clears.
    drive(0, 0);
}

void turnAround() {
    unsigned long elapsed = millis() - stepStart;
    switch (padStep) {
        case ENTER_PAD:
            if (padTravelCm >= 4.0f) { startStep(); padStep = STOP_ON_PAD; }
            else if (elapsed > 2000) stopRun(F("Checkpoint entry timeout"));
            else headingDrive(110, targetHeading);
            break;
        case STOP_ON_PAD:
            drive(0, 0);
            if (elapsed >= 700) {
                startStep();
                targetHeading += 180;
                padStep = TURN_ON_PAD;
            }
            break;
        case TURN_ON_PAD:
            if (rotateTo(targetHeading)) { startStep(); padStep = LEAVE_PAD; }
            else if (elapsed > 5000) stopRun(F("U-turn timeout"));
            break;
        case LEAVE_PAD:
            headingDrive(155, targetHeading);
            if (held(!widePad(), 120)) padStep = OFF_PAD;
            else if (elapsed > 2000) stopRun(F("Checkpoint exit timeout"));
            break;
        case OFF_PAD:
            break;
    }
}

void waitForObstacle() {
    if (!USE_ULTRASONIC) { stopRun(F("Junction trial complete: stopped before first gate")); return; }
    resumeState = state;
    enterState(AVOID);
}

bool serialButton() {
    bool pressed = false;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == 'b' || c == 'B') pressed = true;
        if ((c == 'd' || c == 'D') && (state == WAIT || state == FINISH)) dumpTrace();
    }
    return pressed;
}

struct TraceRecord {
    uint16_t time10ms;
    int16_t angle, leftMotor, rightMotor;
    uint8_t ir[6], phase, step, runState, third;
};
static_assert(sizeof(TraceRecord) == 18, "Trace format changed");
constexpr uint8_t TRACE_COUNT = 32;
constexpr int TRACE_ADDRESS = 6;
static_assert(TRACE_ADDRESS + TRACE_COUNT * sizeof(TraceRecord) <= E2END + 1, "Trace exceeds EEPROM");
TraceRecord traceRing[TRACE_COUNT];
uint8_t traceNext = 0, traceCount = 0;
bool traceSaved = false;
unsigned long lastTrace = 0;

uint16_t traceCrc(uint16_t crc, uint8_t value) {
    crc ^= value;
    for (uint8_t bit = 0; bit < 8; ++bit)
        crc = (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0);
    return crc;
}

void captureTrace() {
    if (state == WAIT || state == FINISH || millis() - lastTrace < 200) return;
    lastTrace = millis();
    TraceRecord &r = traceRing[traceNext];
    r.time10ms = (lastTrace - runStart) / 10;
    r.angle = (int16_t)heading;
    r.leftMotor = commandedLeft;
    r.rightMotor = commandedRight;
    for (uint8_t i = 0; i < 6; ++i) r.ir[i] = constrain(irValue[i], 0, 1000) / 4;
    r.phase = lanePhase; r.step = laneStep; r.runState = state; r.third = thirdSensorOnEdge;
    traceNext = (traceNext + 1) % TRACE_COUNT;
    if (traceCount < TRACE_COUNT) ++traceCount;
}

void saveTrace() {
    if (traceSaved || traceCount == 0) return;
    drive(0, 0);
    digitalWrite(LED_PIN, LOW);
    // Invalidate first; commit last so interrupted writes are rejected.
    EEPROM.update(0, 0);
    EEPROM.update(1, 1);
    EEPROM.update(2, traceCount);
    uint16_t crc = traceCrc(traceCrc(0xFFFF, 1), traceCount);
    int address = TRACE_ADDRESS;
    uint8_t first = (traceNext + TRACE_COUNT - traceCount) % TRACE_COUNT;
    for (uint8_t i = 0; i < traceCount; ++i) {
        const uint8_t *bytes = (const uint8_t *)&traceRing[(first + i) % TRACE_COUNT];
        for (uint8_t j = 0; j < sizeof(TraceRecord); ++j) {
            EEPROM.update(address++, bytes[j]);
            crc = traceCrc(crc, bytes[j]);
        }
    }
    EEPROM.put(3, crc);
    EEPROM.update(0, 0xA7);
    eeprom_busy_wait();
    traceSaved = true;
    digitalWrite(LED_PIN, HIGH);
    Serial.println(F("TRACE SAVED: reconnect USB, send D"));
}

void dumpTrace() {
    uint8_t count = EEPROM.read(2);
    if (EEPROM.read(0) != 0xA7 || EEPROM.read(1) != 1 || count == 0 || count > TRACE_COUNT) {
        Serial.println(F("No saved trace")); return;
    }
    uint16_t stored, crc = traceCrc(traceCrc(0xFFFF, 1), count);
    EEPROM.get(3, stored);
    for (int i = 0; i < count * (int)sizeof(TraceRecord); ++i)
        crc = traceCrc(crc, EEPROM.read(TRACE_ADDRESS + i));
    if (crc != stored) { Serial.println(F("Trace checksum failed")); return; }
    Serial.println(F("ms,state,phase,step,heading,ir0,ir1,ir2,ir3,ir4,ir5,left,right,third"));
    for (uint8_t i = 0; i < count; ++i) {
        TraceRecord r;
        EEPROM.get(TRACE_ADDRESS + i * sizeof(TraceRecord), r);
        Serial.print((unsigned long)r.time10ms * 10); Serial.print(',');
        Serial.print(r.runState); Serial.print(','); Serial.print(r.phase); Serial.print(',');
        Serial.print(r.step); Serial.print(','); Serial.print(r.angle);
        for (uint8_t j = 0; j < 6; ++j) { Serial.print(','); Serial.print(r.ir[j] * 4U); }
        Serial.print(','); Serial.print(r.leftMotor); Serial.print(','); Serial.print(r.rightMotor);
        Serial.print(','); Serial.println(r.third);
    }
    Serial.println(F("END TRACE"));
}

// ---- The state machine ----------------------------------------------------
void setup() {
    zumoBegin();
    for (uint8_t i = 0; i < 6; ++i) {
        irWhite[i] = SIMULATION ? SIM_WHITE[i] : REAL_WHITE[i];
        irBlackUs[i] = SIMULATION ? SIM_BLACK[i] : REAL_BLACK[i];
        if (irBlackUs[i] <= irWhite[i] || irBlackUs[i] > IR_TIMEOUT_US) {
            stopRun(F("Invalid IR calibration range"));
            return;
        }
    }
    enterState(WAIT);
    if (!beginGyro()) stopRun(F("LSM6DS33 at 0x6B not found"));
}

void loop() {
    if (state == FINISH) { serialButton(); return; }
    unsigned long now = millis();
    unsigned long stepMs = now - lastControl;
    if (stepMs < 10 || state == FINISH) return;
    lastControl = now;
    dt = stepMs * 0.001f;
    bool serialPressed = serialButton();
    bool pressed = buttonPressed() || serialPressed;
    if (state != WAIT && (pressed || now - runStart >= 89000)) {
        stopRun(pressed ? F("Stopped by user") : F("Run timeout"));
        return;
    }
    if (!updateGyro(state == WAIT, timeInState())) { stopRun(F("IMU read failed")); return; }
    float distance = forwardDistance(dt);
    if (state == LANE && laneStep == BETWEEN_EDGES && lanePhase == 0) firstStraightCm += distance;
    if (state == LANE && laneStep == CENTER_ON_JUNCTION) junctionTravelCm += distance;
    if ((state == LANE && laneStep == FIND_ENTRY_LINE) || state == OBSTACLES || state == AVOID || state == RETURN)
        sectionTravelCm += distance;
    if ((state == OBSTACLES || state == RETURN) && gateWaitCompleted &&
        sectionTravelCm - gateClearTravelCm >= 30.0f)
        gateWaitCompleted = false;
    if (state == CHECKPOINT && padStep == ENTER_PAD) padTravelCm += distance;
    if (state == RETURN && commandedLeft > 0 && commandedRight > 0) returnDriveMs += stepMs;
    irUpdate();
    if (USE_ULTRASONIC) usUpdate();
    captureTrace();

    switch (state) {
        case WAIT:
            drive(0, 0);
            if (!CALIBRATION_MODE && !startDelayActive && pressed && biasCount >= 200) {
                startDelayActive = true;
                startDelayBegin = now;
                Serial.println(F("Starting in 1 second"));
            }
            if (startDelayActive && now - startDelayBegin >= START_DELAY_MS) {
                runStart = now;
                digitalWrite(LED_PIN, LOW);
                enterState(LANE);
            }
            break;
        case LANE:
            keepLane();
            if (zoneReached()) enterState(OBSTACLES);
            break;
        case OBSTACLES:
            if (obstacleClose()) waitForObstacle();
            else if (checkpointReached()) enterState(CHECKPOINT);
            else keepLane();
            break;
        case AVOID:
            goAround();
            if (pathClear()) {
                gateWaitCompleted = true;
                gateClearTravelCm = sectionTravelCm;
                enterState(resumeState);
            } else if (timeInState() > 14000) stopRun(F("Obstacle did not clear"));
            break;
        case CHECKPOINT:
            turnAround();
            if (uTurnDone()) {
                sectionTravelCm = 0;
                gateWaitCompleted = false;
                gateClearTravelCm = -100.0f;
                enterState(RETURN);
            }
            break;
        case RETURN:
            if (obstacleClose()) waitForObstacle();
            else {
                keepLane();
                if (finishReached()) enterState(FINISH);
            }
            break;
        case FINISH:
            drive(0, 0);
            break;
    }

    // Uncomment to print the ultrasonic distance (cm) while debugging.
    // Serial.println(usCm);
}
