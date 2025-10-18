#include "M2Machine.h"

// Wall-clock time helper (seconds since epoch) for logs
static inline double system_time_sec() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(system_clock::now().time_since_epoch()).count();
}
// Transition guard: calibration finished -> leave CalibState
static bool endCalib(StateMachine& sm) {
    return (sm.state<M2CalibState>("CalibState"))->isCalibDone();
}

// Transition guard: Standby -> ProbMove when UI sends BGIN
static bool toProbOnBtn12(StateMachine& SM){
    auto& sm = static_cast<M2Machine&>(SM);

    if (sm.UIserver && sm.UIserver->isCmd()) {
        std::string cmd; std::vector<double> v;
        sm.UIserver->getCmd(cmd, v);

        // MODIFIED: "STRT_PROB" -> "BGIN"
        if (cmd == "BGIN") {
            sm.UIserver->clearCmd();
            sm.UIserver->sendCmd(std::string("OK"));
            spdlog::info("[TRANS] accepting BGIN -> toProb");
            return true;
        }

        else {
            sm.UIserver->clearCmd();
        }
    }
    return false;
}

// Transition guard: ProbMove finished -> Standby
static bool probMoveFinished(StateMachine& sm){
    return sm.state<M2ProbMoveState>("ProbMoveState")->isFinished();
}

M2Machine::M2Machine() {
    setRobot(std::make_unique<RobotM2>("M2_MELB"));

    // Register states
    addState("CalibState",   std::make_shared<M2CalibState>(robot()));
    addState("StandbyState", std::make_shared<M2StandbyState>(robot(),this));
    // The constructor call remains the same
    addState("ProbMoveState",std::make_shared<M2ProbMoveState>(robot(),this));

    // Wire transitions
    addTransition("CalibState", &endCalib,"StandbyState");
    // MODIFIED: Transition name is updated for clarity
    addTransition("ProbMoveState", &probMoveFinished, "StandbyState");
    addTransition("StandbyState", &toProbOnBtn12, "ProbMoveState");

    setInitState("CalibState");
}
M2Machine::~M2Machine() {
}

void M2Machine::init() {
    spdlog::debug("M2Machine::init()");
    if (robot()->initialise()) {
        // Basic machine-wide CSV logger (position/velocity/force)
        logHelper.initLogger("M2MachineLog", "logs/M2Machine.csv", LogFormat::CSV, true);
        logHelper.add(runningTime(),                 "Time (s)");
        logHelper.add(robot()->getEndEffPosition(),  "Position");
        logHelper.add(robot()->getEndEffVelocity(),  "Velocity");
        logHelper.add(robot()->getEndEffForce(),     "Force");
        logHelper.startLogger();
        // Initialise a default session id with epoch seconds if Unity hasn't set one yet
        if (sessionId == "UNSET") {
            auto now = std::chrono::system_clock::now();
            auto secs = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
            sessionId = std::to_string(secs);
        }
        // UI server used to exchange simple commands and telemetry with Unity
        UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.8.104");
    } else {
        spdlog::critical("Failed robot initialisation. Exiting...");
        std::raise(SIGTERM);
    }
}

void M2Machine::end() {
    if (running() && UIserver) 
        UIserver->closeConnection();
    StateMachine::end();
}

void M2Machine::hwStateUpdate() {
    // Drive the underlying state machine and push current state to UI
    StateMachine::hwStateUpdate();
    if (UIserver) UIserver->sendState();
}
/*
 * SPDX-License-Identifier: MIT
 *
 * M2 Probabilistic Move Controller – Machine Setup and Transitions
 *
 * Copyright (c) 2025  Tiancheng Yang
 * Affiliation: University of Melbourne
 *
 * License: This file is licensed under the MIT License (see LICENSE at repo root).
 *
 * Data and Usage Notes:
 * - Establishes UI server (FLNLHelper) for Unity integration.
 * - Initializes machine-wide CSV logging to `logs/M2Machine.csv`.
 * - See M2States.* for state logic and preload detection details.
 */
