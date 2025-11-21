/*
 * =====================================================================================
 *
 *      Project:   M2 Human–Robot Interaction Experiment Framework
 *      Module:    M2Machine / M2States
 *      Purpose:   State machine implementation for M2 robot control, trial logic,
 *                 effort computation, deterministic perturbation scheduling, and
 *                 Unity interface synchronization.
 *
 *      Author:    Tiancheng (Gavin) Yang
 *      Student ID: 1456070
 *      Affiliation: The University of Melbourne
 *      Contact:   tianchengy2@student.unimelb.edu.au
 *
 * -------------------------------------------------------------------------------------
 *      Copyright (c) 2025 Tiancheng Yang, The University of Melbourne
 *      License: MIT
 *
 *      Permission is hereby granted, free of charge, to any person obtaining a copy
 *      of this software and associated documentation files (the “Software”), to deal
 *      in the Software without restriction, including without limitation the rights
 *      to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *      copies of the Software, and to permit persons to whom the Software is
 *      furnished to do so, subject to the following conditions:
 *
 *      The above copyright notice and this permission notice shall be included in all
 *      copies or substantial portions of the Software.
 *
 *      THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *      IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *      FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *      AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *      LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *      OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *      SOFTWARE.
 *
 * =====================================================================================
 */

 
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
        // UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.8.104");
        // UIserver = std::make_shared<FLNLHelper>(*robot(), "169.254.52.239");
        // UIserver = std::make_shared<FLNLHelper>(*robot(), "192.168.10.2");
        UIserver = std::make_shared<FLNLHelper>(*robot(), "0.0.0.0");
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

// void M2Machine::hwStateUpdate() {
//     // Drive the underlying state machine and push current state to UI
//     StateMachine::hwStateUpdate();
//     if (UIserver) UIserver->sendState();
// }

// void M2Machine::hwStateUpdate() {
//     // Drive the underlying state machine and push current state to UI
//     StateMachine::hwStateUpdate();
//     static double lastSend = 0.0;
//     double now = system_time_sec();
//     if (UIserver){
//         if(now - lastSend > 0.005) {// 200Hz
//             UIserver->sendState();
//         }
//             lastSend = system_time_sec();
// }}

// void M2Machine::hwStateUpdate() {
//     // Drive the underlying state machine and push current state to UI
//     StateMachine::hwStateUpdate();

//     static auto last = std::chrono::steady_clock::now();  // 上次发送时间
//     auto now = std::chrono::steady_clock::now();

   
//     constexpr double interval_ms = 25; // 40Hz

//     if (std::chrono::duration<double, std::milli>(now - last).count() >= interval_ms) {
//         if (UIserver) {
//             UIserver->sendState();
//         }
//         last = now;
//     }
// }




// void M2Machine::hwStateUpdate() {
//     // 每次循环先检查是否掉线，掉线则等待新连接
//     if (!UIserver->isConnected()) 
//         {
//             spdlog::critical("UI disconnected. Waiting for new connection...");
//             UIserver->reconnect();
//             spdlog::info("UI reconnected.");
//     }

//     StateMachine::hwStateUpdate();

//     static auto last = std::chrono::steady_clock::now();
//     auto now = std::chrono::steady_clock::now();
//     constexpr double interval_ms = 25; // 40Hz

//     if (std::chrono::duration<double, std::milli>(now - last).count() >= interval_ms) {
//         if (UIserver) {
//             UIserver->sendState();
//         }
//         last = now;
//     }
// }


// 成员或静态局部
static bool connected = false;
static auto lastCheck = std::chrono::steady_clock::now();

void M2Machine::hwStateUpdate() {
    auto now = std::chrono::steady_clock::now();

    // 每 1s 检查一次
    if (UIserver && std::chrono::duration<double,std::milli>(now - lastCheck).count() > 1000.0) {
        connected = UIserver->isConnected();
        if (!connected) {
            spdlog::critical("UI down, waiting reconnect...");
            UIserver->reconnect();       // 阻塞等待新客户端
            connected = UIserver->isConnected();
            spdlog::info("UI reconnected");
        }
        lastCheck = now;
    }

    StateMachine::hwStateUpdate();

    // 仅在已连接时发状态
    static auto lastSend = std::chrono::steady_clock::now();
    if (connected && std::chrono::duration<double,std::milli>(now - lastSend).count() >= 25.0) {
        UIserver->sendState();
        lastSend = now;
    }
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
