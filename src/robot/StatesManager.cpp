#include "StatesManager.h"

#include <TinyShell.h>

#include <cstdio>
#include <string>

States* States::instance_ = nullptr;

// conversion from stateName to string for logging purposes
static const char* stateToString(stateName state) {
    switch (state) {
        case NONE:          return "NONE";
        case SETUP:         return "SETUP";
        case WAIT:          return "WAIT";
        case CALIBRATE:     return "CALIBRATE";
        case DEBUG:         return "DEBUG";
        case RUN:           return "RUN";
        case FINISH:        return "FINISH";
        case TELEMETRY:     return "TELEMETRY";
        case ERROR:         return "ERROR";
        default:            return "UNKNOWN";
    }
}

stateName States::process_transition(stateName currentState, uint8_t buttons) {
    // A request from "state set" stands in for the button lookup on this pass
    // and then goes through exactly the same gates a real button press does.
    // Consumed here, in the single funnel every state's transition passes
    // through, so there is no second path a remote caller could take to skip
    // the checks below.
    const uint8_t requested = StateMachine::take_request();

    // "state lock": hold the machine still while it is being reconfigured
    // over the radio. A pending request is dropped, not queued -- an unlock
    // that suddenly jumps somewhere the operator asked for minutes earlier is
    // worse than making them ask again.
    if (StateMachine::is_locked()) {
        if (requested != NONE) {
            robot_.logger.insert_logf(
                logType::WARN,
                "state locked: request to %s dropped (use 'state unlock')",
                StateMachine::stateToString(requested));
        }
        return currentState;
    }

    stateName nextState = currentState;
    if (requested != NONE) {
        // Deliberately not validated against transitionTable: the table is the
        // BUTTON policy, and a request exists precisely to reach states the
        // buttons cannot reach from here (e.g. straight into TELEMETRY). The
        // safety gates below are what constrain it.
        nextState = static_cast<stateName>(requested);
    } else {
        for (int i = 0; i < NUM_TRANSITIONS; i++)
            if (transitionTable[i].currentState == currentState)
                if (buttons & transitionTable[i].buttonMask)
                    nextState = transitionTable[i].nextState;
    }

    // The SD card must stay exclusively owned by the PC until safe eject has
    // returned it to the robot. Do not leave the safe DEBUG state before that.
    if (currentState == DEBUG && robot_.usb_storage.is_active()) {
        if (nextState != DEBUG) {
            robot_.logger.insert_log(
                logType::WARN,
                "transition refused: the USB host still owns the SD card; safe-eject it first");
        }
        return DEBUG;
    }

    // Never interrupt a firmware write in progress.
    if (currentState == DEBUG && robot_.ota.is_flashing()) {
        if (nextState != DEBUG) {
            robot_.logger.insert_log(
                logType::WARN,
                "transition refused: an OTA firmware write is in progress");
        }
        return DEBUG;
    }

    if (currentState == DEBUG && nextState != DEBUG) {
        robot_.cancelDebugTests();
    }

    #if defined(LOG_ALL) || defined(LOG_INFO)
    if (nextState != currentState)
        // log message
        robot_.logger.insert_logf(logType::INFO, "state changed: %s -> %s", stateToString(currentState), stateToString(nextState));
    #endif

    return nextState; 
}

stateName States::go_to(stateName currentState, stateName nextState) {
    // log message
    #if defined(LOG_ALL) || defined(LOG_INFO)
        robot_.logger.insert_logf(logType::INFO, "state changed: %s -> %s", stateToString(currentState), stateToString(nextState));
    #endif

    return nextState;
}

void States::register_shell_commands(TinyShell& shell, Logger& logger) {
    // "state": read and command the state machine directly. Registered here,
    // not in BallyRobotShell.cpp, because the transition POLICY lives here --
    // transitionTable[] and process_transition() are this file's, and "state
    // table" would otherwise force the composition root to reach into
    // src/robot for application policy it has no other reason to know.
    //
    // StateMachine (lib/) holds the request, the lock and the history ring;
    // this module is only the shell face of them.
    shell.create_module("state", "Read and command the state machine");

    shell.add([&logger]() -> uint8_t {
        const uint8_t pending = StateMachine::pending_request();
        logger.insert_logf(
            logType::INFO, "state=%s locked=%d pending_request=%s",
            StateMachine::stateToString(
                StateMachine::current_state.load(std::memory_order_acquire)),
            StateMachine::is_locked() ? 1 : 0,
            StateMachine::stateToString(pending));
        return RESULT_OK;
    }, "get", "Current state, lock status and any pending request", "state");

    shell.add([&logger](std::string name) -> uint8_t {
        const uint8_t target = StateMachine::stateFromString(name.c_str());
        if (target == NONE) {
            logger.insert_logf(
                logType::ERRO,
                "unknown state '%s' (SETUP|WAIT|CALIBRATE|DEBUG|RUN|FINISH|TELEMETRY|ERROR)",
                name.c_str());
            return RESULT_ERROR;
        }

        if (StateMachine::is_locked()) {
            logger.insert_log(logType::ERRO,
                              "state is locked; run 'state unlock' first");
            return RESULT_ERROR;
        }

        if (!StateMachine::request_state(target)) {
            logger.insert_logf(logType::ERRO, "state %s cannot be requested",
                               StateMachine::stateToString(target));
            return RESULT_ERROR;
        }

        // Requested, not applied: process_transition() consumes it on the next
        // check (timers.delay_flags, 250 ms by default) and may still refuse it
        // -- USB owning the SD card or an OTA write in progress both win. Say
        // "requested" so a refusal further down does not read as a lie here.
        logger.insert_logf(logType::INFO, "requested_state=%s pending=1",
                           StateMachine::stateToString(target));
        return RESULT_OK;
    }, "set", "Request a transition to a state by name", "state");

    shell.add([&logger]() -> uint8_t {
        std::string out = "button transition table (mask is a button bitmask)\n";
        char line[96];
        for (int i = 0; i < NUM_TRANSITIONS; ++i) {
            std::snprintf(line, sizeof(line), "%d from=%s mask=0x%02X to=%s\n", i,
                          StateMachine::stateToString(transitionTable[i].currentState),
                          static_cast<unsigned>(transitionTable[i].buttonMask),
                          StateMachine::stateToString(transitionTable[i].nextState));
            out += line;
        }
        // CALIBRATE and TELEMETRY are absent on purpose (see the note next to
        // transitionTable): both return to WAIT from inside their own action
        // function, so no button ever moves them.
        out += "note: CALIBRATE and TELEMETRY self-transition to WAIT from their action function";
        logger.insert_log(logType::INFO, out.c_str());
        return RESULT_OK;
    }, "table", "List the button-driven transition table", "state");

    shell.add([&logger]() -> uint8_t {
        StateMachine::set_locked(true);
        logger.insert_log(logType::WARN, "locked=1 transitions are held");
        return RESULT_OK;
    }, "lock", "Hold the machine in its current state (buttons included)", "state");

    shell.add([&logger]() -> uint8_t {
        StateMachine::set_locked(false);
        logger.insert_log(logType::INFO, "locked=0 transitions resumed");
        return RESULT_OK;
    }, "unlock", "Allow transitions again", "state");

    shell.add([&logger]() -> uint8_t {
        StateMachine::TransitionRecord records[StateMachine::kHistoryDepth];
        const size_t count =
            StateMachine::history(records, StateMachine::kHistoryDepth);
        if (count == 0) {
            logger.insert_log(logType::INFO, "transitions=0");
            return RESULT_OK;
        }

        std::string out;
        char line[80];
        for (size_t i = 0; i < count; ++i) {
            std::snprintf(line, sizeof(line), "%u from=%s to=%s uptime_ms=%lu\n",
                          static_cast<unsigned>(i),
                          StateMachine::stateToString(records[i].from),
                          StateMachine::stateToString(records[i].to),
                          static_cast<unsigned long>(records[i].uptime_ms));
            out += line;
        }
        logger.insert_log(logType::INFO, out.c_str());
        return RESULT_OK;
    }, "history", "Last transitions, newest first", "state");
}
