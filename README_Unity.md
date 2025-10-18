# Unity Front-End for M2 Experiment

**Author:** Tiancheng (Gavin) Yang  
**Affiliation:** The University of Melbourne  
**License:** MIT

---

## Overview
This Unity module provides the user interface and experimental logic for controlling the **M2 robot** during human–robot interaction experiments. It communicates with the M2 controller via TCP to send commands, display trial states, and collect subject identifiers.

---

## Key Features
- Real-time UI for experiment control
- Countdown and rest timers (e.g., 3–2–1 start and 60 s rest)
- Mode selection (`v1`, `v2`)
- Probability slider and effort display
- Automatic session grouping (10 trials per session)
- Support for numeric `SID` up to 64-bit integers

---

## Communication Protocol
| Command | Description |
|----------|--------------|
| `BGIN` | Begin new session (enter ProbMove mode) |
| `STRT` | Start one trial |
| `S_PB` | Set trial probability (0–1) |
| `S_MD` | Select mode (v1 / v2) |
| `S_SID` | Send subject/session ID |
| `TREN` | Received from M2 to indicate trial end |

---

## Experiment Flow
1. **Begin Session** → `BGIN`
2. **Wait at Start** (state: WAIT_START)
3. **Send SID + Parameters** (`S_SID`, `S_PB`, etc.)
4. **Countdown (3–2–1)** → then send `STRT`
5. **Trial Execution**
6. **Inter-trial pause** (5 s)
7. **Rest between sessions** (60 s)
8. **Repeat until 10 trials complete**

---

## Notes
- Parameters sent in Standby are ignored by M2.
- All updates must be applied between trials (WAIT_START).
- SID can be any 64-bit integer; strings are not yet supported.
- UI supports full automation of 10-trial blocks.

---

## License
This project is licensed under the MIT License.  
© 2025 Tiancheng Yang, The University of Melbourne.