
#ifndef M2_MACHINE_H
#define M2_MACHINE_H

#include "RobotM2.h"
#include "StateMachine.h"
#include "FLNLHelper.h"

// State Classes
#include "M2States.h"

// Top-level state machine wrapper for M2 robot runtime.
// - Owns the RobotM2 instance and the simple TCP/UDP UI helper.
// - Wires up states and transitions and proxies periodic hw updates.
class M2Machine : public StateMachine {
public:
    M2Machine();
    ~M2Machine();

    // Set up robot, logging, and UI server
    void init();
    // Tear down UI and state machine
    void end();

    // Per-cycle hardware/state update tick
    void hwStateUpdate();

    RobotM2* robot() { return static_cast<RobotM2*>(_robot.get()); } // typed getter

    std::shared_ptr<FLNLHelper> UIserver = nullptr; // UI/command server
    // Session/subject identifier provided by Unity
    std::string sessionId = "UNSET";
};

#endif /* M2_MACHINE_H */
/*
 * SPDX-License-Identifier: MIT
 *
 * M2 Probabilistic Move Controller – State Machine Wrapper
 *
 * Copyright (c) 2025  Tiancheng Yang
 * Affiliation: University of Melbourne
 *
 * License: This file is licensed under the MIT License (see LICENSE at repo root).
 *
 * Data and Usage Notes:
 * - This software logs experimental telemetry under `logs/` (CSV files).
 * - Logged data may contain session identifiers; anonymize before sharing.
 * - UI commands originate from the Unity frontend via FLNLHelper.
 * - For preload detection parameters, see commands `S_PLT` (threshold, N)
 *   and `S_PLW` (window, s). Default: 3.0 N and 0.200 s.
 */
