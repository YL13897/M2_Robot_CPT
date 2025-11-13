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


#ifndef M2_STATES_H_DEF
#define M2_STATES_H_DEF

#include "RobotM2.h"
#include "State.h"
#include "StateMachine.h"
#include <random>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <deque>
using namespace std;


double timeval_to_sec(struct timespec *ts);

class M2Machine;


// Base state with standardized entry/during/exit and console banners
class M2TimedState : public State {
   protected:

    RobotM2 *robot;                               /*<!Pointer to state machines robot object*/

    M2TimedState(RobotM2 *M2, const char *name = NULL): State(name), robot(M2){};
   private:
    void entry(void) final {
        std::cout
        << "==================================" << std::endl
        << " STARTING  " << getName() << std::endl
        << "----------------------------------" << std::endl
        << std::endl;

        //Actual state entry
        entryCode();
    };
    void during(void) final {
        //Actual state during
        duringCode();
    };
    void exit(void) final {
        exitCode();
        std::cout
        << "----------------------------------" << std::endl
        << "EXIT "<< getName() << std::endl
        << "==================================" << std::endl
        << std::endl;
    };

   public:
    virtual void entryCode(){};
    virtual void duringCode(){};
    virtual void exitCode(){};
    
};

// Joint stop seeking + encoder calibration; exits when calibrated
class M2CalibState : public M2TimedState {

   public:
    M2CalibState(RobotM2 *M2, const char *name = "M2 Calib State"):M2TimedState(M2, name){};

    void entryCode(void);
    void duringCode(void);
    void exitCode(void);

    bool isCalibDone() {return calibDone;}

   private:
     VM2 stop_reached_time;
     bool at_stop[2];
     bool calibDone=false;
};

// Transparent idle (zero commanded force) with light CSV logging
class M2StandbyState : public M2TimedState {
public:
    M2StandbyState(RobotM2* M2, M2Machine* mach, const char* name = "M2 Standby")
        : M2TimedState(M2, name), machine(mach) {}

    void entryCode() override;
    void duringCode() override;
    void exitCode() override;


private:
    M2Machine* machine = nullptr;

    // Logging helpers/fields
    std::ofstream standbyCsv_;
    int  standbyLogEveryN_ = 10;  // log every 10 iterations to reduce I/O
    int  standbyIter_ = 0;
    bool standbyRecording_ = true; // set false to disable logging

    void openStandbyCSV_();
    void closeStandbyCSV_();
    void writeStandbyCSV_(double t, double sys_t, const std::string& sid,
                          const VM2& pos, const VM2& vel,
                          const VM2& fcmd, const VM2& fsense);
};


// Probabilistic move block: TO_A -> WAIT_START (preload check) -> TRIAL
// Handles UI commands, scoring, deterministic LEFT/UP schedule, and CSV logs
class M2ProbMoveState : public M2TimedState {
public:
    M2ProbMoveState(RobotM2* M2, M2Machine* mach, const char* name="M2 Probabilistic Move");

    void entryCode() override;
    void duringCode() override;
    void exitCode() override;

    bool isFinished() const { return finishedFlag; }

    // --- Perturbation force arrays loaded from CSV ---
    std::vector<double> upPerturbForce;
    std::vector<double> leftPerturbForce;
    size_t perturbIndex = 0;
    bool injectingUp = false;
    bool injectingLeft = false;
    // --- WAIT_START hold-at-A configuration ---
    bool waitHoldLatched_ = false;  
    double k_hold = 2000.0;           
    double d_hold = 10.0;            

    // --- Perturbation CSV loading functions ---
    std::vector<double> loadColumnFromCSV(const std::string& path, int colIndex, double tStart, double tEnd);
    void loadPerturbationForces();

public:
    // experiment config
    VM2 A{0.45, 0.002};
    VM2 C{0.45, 0.302};

    double epsA = 0.05;
    double epsC = 0.05;
    
    double lastTrpsT_ = -1.0;
    double trpsMinInterval_ = 0.5; // 20Hz

    std::vector<VM2> trialEndPositions_;   
    bool sendPosOnlyOnTimeout_ = false;    
    
    double k = 50;
    double d = 6;

    double robotForceMagUp  = 35;
    double robotForceMagLeft= 17.5;
    
    double internalForceDur  = 1.2;
    double trialMaxTime      = 1.2;

    VM2    internalForce     = VM2::Zero();

    double userForceScale   = 5;
    double forceSaturation   = 80.0;
    double Dv               = 5.0;

    double rampUp       = 0.5;
    double rampDown     = 0.2;

    double probLeft = 0.5;

    bool   enablePIDToA = true;
    double KpToA = 6.0;
    double KiToA = 35.0;
    double KdToA = 1.0;
    VM2    iErrToA = VM2::Zero();
    double iToA_max = 15.0;

    // --- Meta parameters received from UI ---
    int    meta_scoreMode   = 1;
    int    meta_targetSucc  = 10;
    int    meta_maxTrials   = 10;

    double V2_Smax = 110.0;

    // ToA related variables
    double holdTimeA  = 0.25;
    double epsA_hold  = 0.06;
    double inBandSince = 0.0;
    VM2    Xi;
    double T_toA  = 2.0;
    double t0_toA = 0.0;

    M2Machine* machine = nullptr;

    // Helper methods (no changes needed)
    VM2 impedance(const VM2& X0, const VM2& X, const VM2& dX, const VM2& dXd = VM2::Zero());
    VM2 readUserForce();
    void decideInternalForceDirection();
    void resetToAPlan(const VM2& Xnow);
    void resetToAIntegrators();
    void writeCSV(double t, const VM2& pos, const VM2& vel, const VM2& fInternal, const VM2& fUser, double effort);
    void applyForce(const VM2& F);
    bool startTrialSignal();


private:
    void handleInput();
    void openCSV();
    // Deterministic schedule for LEFT/UP within a 10-trial block: -1=LEFT, +1=UP
    std::vector<int> trialSchedule_;

    size_t trialIdx_ = 0;
    void buildDeterministicSchedule();

    // MERGED: Re-introducing enum for internal state management
    enum Phase {
        TO_A,
        WAIT_START,
        TRIAL
    };
    Phase currentPhase;

    // MERGED: Flags to simulate entryCode() for each phase
    bool initToA = true;
    bool initTrial = true;
    bool pendingStart = false;  // captured STRT; consumed only in WAIT_START
    bool betweenTrials = false; // true only in WAIT_START between trials; allows S_MD/S_MT/S_TS

    // MERGED: Variables from M2TrialState are now here
    double trialStartTime = 0.0;
    double effortIntegral = 0.0;

    double rawEffortIntegral = 0.0;

    double baselineImpulseN = 0.0;
    
    bool finishedFlag = false;

    // Scoring-related state variables
    enum ScoringMode { V1_COUNT_SUCCESS, V2_EFFORT_DISTANCE };
    ScoringMode currentMode;
    int successfulTrials = 0;
    int totalTrialsV1 = 0;
    double totalScoreV2 = 0.0;
    int totalTrialsV2 = 0;
    // STRT debounce (seconds)
    double lastStrtTime = -1.0;
    double strtMinInterval = 1.0;
    std::mt19937 rng;
    std::ofstream csv;
    // --- Add to M2ProbMoveState (private) ---
    int txSeq_ = 0;

    // --- Preload detection (WAIT_START) ---
    struct WaitSample {
        double t;      // state running() time
        VM2    pos;    // end-eff position
        VM2    vel;    // end-eff velocity
        VM2    force;  // sensed end-eff force
    };
    std::deque<WaitSample> waitBuf_;           // rolling buffer of recent WAIT_START samples
    double preloadThresholdN_ = 3.0;           // adjustable threshold (N), default 3N
    double preloadWindowSec_  = 0.200;         // window (s), default 200ms
    bool   preloadSatisfied_  = false;         // result for the upcoming trial
    std::ofstream preloadWinCsv_;              // raw 200ms window dump
    std::ofstream trialTagsCsv_;               // per-trial tags (preload yes/no)
    void openPreloadCSVs_();
    void closePreloadCSVs_();
    void writePreloadWindow_(int trialIdxForMode, double tNow);
    void writeTrialTag_(int trialIdxForMode, int mode, bool flag, double tNow);

    static bool isPrintableAscii(const std::string& s) {
        for (unsigned char ch : s) {
            if (ch < 0x20 || ch > 0x7E) { 
                if (ch!='\n' && ch!='\r' && ch!='\t') return false;
            }
        }
        return true;
    }

    static std::string hexDump(const std::string& s) {
        std::ostringstream h;
        h.setf(std::ios::hex, std::ios::basefield);
        h.setf(std::ios::uppercase);
        for (unsigned char ch : s) {
            h << std::setw(2) << std::setfill('0') << (int)ch << ' ';
        }
        return h.str();
    }

    void sendUI_(const std::string& msg);
};

#endif
/*
 * SPDX-License-Identifier: MIT
 *
 * M2 Probabilistic Move Controller – State Interfaces
 *
 * Copyright (c) 2025  Tiancheng Yang
 * Affiliation: University of Melbourne
 *
 * License: This file is licensed under the MIT License (see LICENSE at repo root).
 *
 * Data and Usage Notes:
 * - WAIT_START logs raw 200ms preload windows to `logs/PreloadWindow_<session>.csv`.
 * - Per-trial preload tags are written to `logs/TrialTags_<session>.csv`.
 * - Configure preload via `S_PLT` (N) and `S_PLW` (s) UI commands.
 */
