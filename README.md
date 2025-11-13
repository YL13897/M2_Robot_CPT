# M2 Human–Robot Interaction Experiment Framework

Author: Tiancheng (Gavin) Yang  
Affiliation: The University of Melbourne  
License: MIT

---

## Overview
This repository provides a complete experimental stack for the M2 haptic robot:
- M2 controller (C++): real‑time state machine, perturbation scheduling, preload detection, and CSV logging.
- Unity front‑end: operator UI, countdowns, per‑trial control, and session metadata.

Both components communicate over a simple TCP interface to coordinate trials and persist reproducible data for analysis.

---

## Components
- `M2Machine/`: robot runtime and state machine with phases
  - `CalibState` → `StandbyState` → `ProbMoveState`
  - `ProbMoveState` phases: `TO_A` → `WAIT_START` → `TRIAL`
- `Unity-Ui/`: Unity app that drives sessions, countdowns, and parameter updates

---

## Key Features
- Deterministic LEFT/UP perturbation scheduling per 10‑trial block
- Preload detection in `WAIT_START` (last 200 ms, default threshold 3 N on +X)
- Adjustable preload parameters from UI (`S_PLT`, `S_PLW`)
- Two scoring modes (V1 count successes, V2 effort‑distance)
- Robust CSV logging (per‑state and per‑trial)
- Explicit STRT‑gated trials for automation and reproducibility

---

## Quick Start
1. Launch the M2 controller binary.
2. Ensure Unity connects (UI shows link status).
3. Begin a prob‑move session: send `BGIN`.
4. Set parameters between trials (in `WAIT_START`):
   - `S_SID <int>`: session/subject ID (64‑bit integer)
   - `S_MD <1|2>`: scoring mode (1 = V1 count, 2 = V2 effort)
   - `S_TS <n>`: V1 target successes
   - `S_MT <n>`: maximum trials
   - `S_PLT <N>`: preload threshold in Newtons (default 3.0)
   - `S_PLW <s>`: preload window in seconds (default 0.200)
5. Unity performs a 3‑2‑1 countdown and sends `STRT` to start a trial.
6. Review CSVs in `logs/` after runs.

Notes
- Parameter commands are accepted only between trials (`WAIT_START`).
- STRT is debounced; send only once per trial start.

---

## Communication Protocol
Commands (UI → M2):
- `BGIN` — enter ProbMove state (start a block)
- `STRT` — start a trial (consumed in `WAIT_START` when at A)
- `HALT` — end ProbMove and return to Standby
- `RSTA` — reset to `TO_A` (re‑center/hold at A)
- `S_SID <int>` — set session/subject ID
- `S_MD <1|2>` — set scoring mode
- `S_TS <int>` — set target successes (V1)
- `S_MT <int>` — set max trials
- `S_PLT <float>` — set preload threshold (N)
- `S_PLW <float>` — set preload window (s)

Events/Reports (M2 → UI):
- `OK`/`BUSY`/`ERR ARG` — command results
- `TRIAL_BEGIN ... preload=0|1 ...` — trial start summary with preload tag
- `TREN` — trial end summary (time, duration, reached, distance, effort, score, mode, indices)
- `SESS` — session summary (mode, V1 success/trials, V2 score/trials)

---

## Experiment Flow
1. `BGIN` → M2 transitions to `TO_A`, then `WAIT_START` (hold at A).
2. UI updates parameters (only in `WAIT_START`).
3. UI countdown (3‑2‑1) → send `STRT`.
4. M2 checks last 200 ms preload window (X‑force ≥ threshold), logs tag, enters `TRIAL`.
5. Trial runs with internal LEFT/UP perturbation; on end → back to `WAIT_START`.
6. Repeat until stop condition (target successes/max trials) or `HALT`.

---

## Logging
Logs are written to `logs/` and are session‑scoped when applicable.
- `M2Machine.csv` — global runtime (time, position, velocity, force)
- `M2ProbMove_<session>.csv` — per‑tick trial info (pos, vel, internal/user forces, effort, meta)
- `Standby_<session>.csv` — sparse samples while idling
- `PreloadWindow_<session>.csv` — raw rows within the last preload window before STRT
- `TrialTags_<session>.csv` — per‑trial preload tag and window summary

Tip: Session IDs are included in logs; anonymize before sharing.

---

## Data & Privacy
- Logs can contain subject/session identifiers and sensor traces.
- Anonymize/redact IDs and comply with local ethics/data policies before external sharing.

---

## License
MIT License — see `LICENSE`. © 2025 Tiancheng Yang, The University of Melbourne.

---

## Contact
For issues or questions, include:
- Score mode, target successes, max trials
- Preload settings (`S_PLT`, `S_PLW`)
- Relevant log snippets from `logs/`
