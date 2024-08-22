#include "Flags.h"

// initialize static members
bool Flags::btn1 = false;
bool Flags::btn2 = false;
bool Flags::btn3 = false;
bool Flags::left = false;
bool Flags::right = false;

uint32_t Flags::btn1Time = 0;
uint32_t Flags::btn2Time = 0;
uint32_t Flags::btn3Time = 0;
uint32_t Flags::leftTime = 0;
uint32_t Flags::rightTime = 0;

// ---- handle the interrupt service routines ----
void IRAM_ATTR Flags::isrBtn1() {
    if (millis() - btn1Time >= filterTime) {
        btn1Time = millis(); 
        btn1 = true;
    }
}

void IRAM_ATTR Flags::isrBtn2() {
    if (millis() - btn2Time >= filterTime) {
        btn2Time = millis(); 
        btn2 = true;
    }
}

void IRAM_ATTR Flags::isrBtn3() {
    if (millis() - btn3Time >= filterTime) {
        btn3Time = millis(); 
        btn3 = true;
    }
}

void IRAM_ATTR Flags::isrLeft() {
    if (millis() - leftTime >= filterTime) {
        leftTime = millis(); 
        left = true;
    }
}

void IRAM_ATTR Flags::isrRight() {
    if (millis() - rightTime >= filterTime) {
        rightTime = millis(); 
        right = true;
    }
}

// check if any flag has been up for longer than a specified filterTime
void Flags::checkFlagsDuration() {
    uint32_t currentTime = millis();

    if (btn1 && (currentTime - btn1Time >= filterTime))   btn1 = false;
    if (btn2 && (currentTime - btn2Time >= filterTime))   btn2 = false;
    if (btn3 && (currentTime - btn3Time >= filterTime))   btn3 = false;
    if (left && (currentTime - leftTime >= filterTime))   left = false;
    if (right && (currentTime - rightTime >= filterTime)) right = false;
}

void Flags::printFlags() {
    Serial.print("btn1: "); Serial.print(btn1); Serial.print("\t");
    Serial.print("btn2: "); Serial.print(btn2); Serial.print("\t");
    Serial.print("btn3: "); Serial.print(btn3); Serial.print("\t");
    Serial.print("left: "); Serial.print(left); Serial.print("\t");
    Serial.print("right: "); Serial.println(right);
}
