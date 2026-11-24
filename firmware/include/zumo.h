// Term Project 1 - optional Zumo helper functions.
// Everything here is explained in Lecture 2 and Lecture 3.
#pragma once
#include <Arduino.h>

// ---- Pins, as in Lecture 3. The simulator uses the same wiring
const uint8_t RIGHT_DIR = 7, LEFT_DIR = 8, RIGHT_PWM = 9, LEFT_PWM = 10;
const uint8_t BUTTON_PIN = 12, LED_PIN = 13, EMITTER_PIN = 2;
const uint8_t TRIG_PIN = 6, ECHO_PIN = 3;  // ECHO must be D3 (INT1)
const uint8_t IR_PIN[6] = {4, A3, 11, A0, A2, 5};  // left to right

// ---- Motors: -400 ... 400, 20 kHz on Timer1
const int MOTOR_TOP = 400;

inline void setSpeeds(int left, int right) {
    digitalWrite(LEFT_DIR, left < 0 ? HIGH : LOW);
    digitalWrite(RIGHT_DIR, right < 0 ? HIGH : LOW);
    OCR1B = constrain(abs(left), 0, MOTOR_TOP);   // D10
    OCR1A = constrain(abs(right), 0, MOTOR_TOP);  // D9
}

// ---- IR sensors: irValue[i] is 0 (white) ... 1000 (black), irBlack[i] is true on black
const unsigned int IR_TIMEOUT_US = 2000;
const int IR_GO_BLACK = 600, IR_GO_WHITE = 400;
// Put your Lab 1 measurements here: raw times on white and on black, left to right
unsigned int irWhite[6] = {250, 250, 250, 250, 250, 250};
unsigned int irBlackUs[6] = {1500, 1500, 1500, 1500, 1500, 1500};
unsigned int irRaw[6];
int irValue[6];
bool irBlack[6];

inline void irReadRaw() {
    digitalWrite(EMITTER_PIN, HIGH);
    delayMicroseconds(200);
    for (int i = 0; i < 6; i++) {
        pinMode(IR_PIN[i], OUTPUT);
        digitalWrite(IR_PIN[i], HIGH);
    }
    delayMicroseconds(10);
    for (int i = 0; i < 6; i++) {
        pinMode(IR_PIN[i], INPUT);
        irRaw[i] = IR_TIMEOUT_US;
    }
    unsigned long t0 = micros(), t;
    while ((t = micros() - t0) < IR_TIMEOUT_US)
        for (int i = 0; i < 6; i++)
            if (irRaw[i] == IR_TIMEOUT_US && digitalRead(IR_PIN[i]) == LOW)
                irRaw[i] = t;
    digitalWrite(EMITTER_PIN, LOW);
}

inline void irUpdate() {
    irReadRaw();
    for (int i = 0; i < 6; i++) {
        long v = ((long)irRaw[i] - irWhite[i]) * 1000 / (irBlackUs[i] - irWhite[i]);
        irValue[i] = constrain(v, 0, 1000);
        if (!irBlack[i] && irValue[i] > IR_GO_BLACK) irBlack[i] = true;
        if (irBlack[i] && irValue[i] < IR_GO_WHITE) irBlack[i] = false;
    }
}

// ---- Ultrasonic: call usUpdate() in every loop. usCm is the median of 5, 999 = nothing seen
volatile unsigned long usRise = 0, usWidth = 0;
volatile bool usDone = false;
float usCm = 999;
float usHist[5] = {999, 999, 999, 999, 999};
int usIdx = 0;
unsigned long usLastPing = 0;

inline void usEchoIsr() {
    if (digitalRead(ECHO_PIN) == HIGH) {
        usRise = micros();
    } else {
        usWidth = micros() - usRise;
        usDone = true;
    }
}

inline void usUpdate() {
    if (millis() - usLastPing < 60) return;  // one ping every 60 ms
    noInterrupts();
    bool done = usDone;
    unsigned long width = usWidth;
    usDone = false;
    interrupts();
    usHist[usIdx] = done && width < 30000 ? width / 58.0 : 999;
    usIdx = (usIdx + 1) % 5;
    float s[5];
    for (int i = 0; i < 5; i++) s[i] = usHist[i];
    for (int i = 1; i < 5; i++)
        for (int j = i; j > 0 && s[j - 1] > s[j]; j--) {
            float t = s[j]; s[j] = s[j - 1]; s[j - 1] = t;
        }
    usCm = s[2];
    usLastPing = millis();
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
}

// ---- Button: true once per press
inline bool buttonPressed() {
    static bool wasDown = false;
    static unsigned long lastChange = 0;
    bool down = digitalRead(BUTTON_PIN) == LOW;
    if (down != wasDown && millis() - lastChange > 20) {
        wasDown = down;
        lastChange = millis();
        return down;
    }
    return false;
}

// ---- Start everything. Call once in setup()
inline void zumoBegin() {
    Serial.begin(115200);
    pinMode(RIGHT_DIR, OUTPUT);
    pinMode(LEFT_DIR, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(EMITTER_PIN, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    TCCR1A = _BV(COM1A1) | _BV(COM1B1);  // Timer1 mode 8, prescaler 1
    TCCR1B = _BV(WGM13) | _BV(CS10);
    ICR1 = MOTOR_TOP;
    OCR1A = 0;
    OCR1B = 0;
    pinMode(RIGHT_PWM, OUTPUT);
    pinMode(LEFT_PWM, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), usEchoIsr, CHANGE);
}
