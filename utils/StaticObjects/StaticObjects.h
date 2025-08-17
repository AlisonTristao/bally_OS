#ifndef STATIC_OBJECTS
#define STATIC_OBJECTS

// autor: Alison Tristão
// email: AlisonTristao@hotmail.com

#include <Arduino.h>
#include <Pinout.h> 
 
// custom libraries
#include <ArraySensor.h>
#include <Encoder.h>
#include <HBridge.h>
#include <Control.h>
#include <TinyShell.h> 

using namespace std;

class StaticObjects {
public:
    // static objects
    static ArraySensor array_sensor;
    static Encoder encoder_left;
    static Encoder encoder_right;
    static HBridge motor_left;
    static HBridge motor_right;
    static Control control;
    static TinyShell shell;

    static void init_objects() {
        // initialize all static objects
        array_sensor.init(INIT_MUX);
        encoder_left.init();
        encoder_right.init();
        motor_left.init();
        motor_right.init();
    }
};

#endif