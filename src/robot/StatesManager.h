#ifndef STATES_H
#define STATES_H

#include <BallyRobot.h>

// this struct relates the current state, th buttons and the next state
struct Transition {
    stateName currentState;
    uint8_t buttonMask;
    stateName nextState; 
};

// This table defines the transitions between states based on the current state and the buttons pressed
inline constexpr Transition transitionTable[] = {
// CURRENT STATE | BUTTON (CONDITION) | NEXT STATE
// ------------------ init ---------------------------
{ SETUP,            (1 << BIT_0),         WAIT },      

// --- wait square ---
{ WAIT,             (1 << BIT_0),         RUN },       
{ WAIT,             (1 << BIT_1),         CALIBRATE }, 
{ WAIT,             (1 << BIT_2),         DEBUG },     

// --- convergence on finish ---
{ RUN,              (1 << BIT_0),         FINISH },
{ DEBUG,            (1 << BIT_0),         FINISH },

// --- go to telemetry ---
{ FINISH,           (1 << BIT_1),         TELEMETRY },

// --- return to wait ---
{ FINISH,           (1 << BIT_0),         WAIT },

// --- fail-safe: any button dumps whatever telemetry is available before a reboot ---
{ ERROR,            (1 << BIT_0) | (1 << BIT_1) | (1 << BIT_2), TELEMETRY },
};
// note: CALIBRATE and TELEMETRY are not listed above — both self-transition
// back to WAIT from inside their own action function (see
// calibrate_function()/telemetry_function()), the same one-shot pattern as
// SETUP.
inline constexpr int NUM_TRANSITIONS = sizeof(transitionTable) / sizeof(Transition);

class States {
public:
    static States& getInstance() {
        static States instance; // Criado apenas uma vez com segurança
        return instance;
    }

    /**
     * @brief Register the "state" shell module (get/set/table/lock/unlock/
     * history).
     *
     * Lives here rather than in the ROBOT composition root because the
     * transition policy is this file's: transitionTable[] and
     * process_transition() are what "state table" prints and what "state set"
     * feeds. Called from main.cpp, which already knows both this object and
     * the shell, so utils/BallyRobot never has to include src/robot.
     */
    void register_shell_commands(TinyShell& shell, Logger& logger);

private:
// constructor
    States() : 
        state1(SETUP,      [](){ return instance_->setup_function(); },     [](uint8_t buttons){ return instance_->process_transition(SETUP, buttons); }),
        state2(WAIT,       [](){ return instance_->wait_function(); },      [](uint8_t buttons){ return instance_->process_transition(WAIT, buttons); }),
        state3(CALIBRATE,  [](){ return instance_->calibrate_function(); }, [](uint8_t buttons){ return instance_->process_transition(CALIBRATE, buttons); }),
        state4(DEBUG,      [](){ return instance_->debug_function(); },     [](uint8_t buttons){ return instance_->process_transition(DEBUG, buttons); }),
        state5(RUN,        [](){ return instance_->run_function(); },       [](uint8_t buttons){ return instance_->process_transition(RUN, buttons); }),
        state6(FINISH,     [](){ return instance_->finish_function(); },    [](uint8_t buttons){ return instance_->process_transition(FINISH, buttons); }),
        state7(TELEMETRY,  [](){ return instance_->telemetry_function(); }, [](uint8_t buttons){ return instance_->process_transition(TELEMETRY, buttons); }),
        state8(ERROR,      [](){ return instance_->error_function(); },     [](uint8_t buttons){ return instance_->process_transition(ERROR, buttons); }) 
        {
            instance_ = this;
        };

    // states
    stateName calibrate_function();
    stateName debug_function();
    stateName error_function();
    stateName finish_function();
    stateName run_function();
    stateName setup_function();
    stateName telemetry_function();
    stateName wait_function();

    // states 
    StateMachine state1; // setup
    StateMachine state2; // wait
    StateMachine state3; // calibrate
    StateMachine state4; // debug
    StateMachine state5; // run
    StateMachine state6; // finish
    StateMachine state7; // telemetry
    StateMachine state8; // error

    // this isntance 
    static States* instance_;

    // local robot
    ROBOT& robot_ = ROBOT::getInstance();

    stateName process_transition(stateName currentState, uint8_t buttons);
    stateName go_to(stateName currentState, stateName nextState);
};

#endif