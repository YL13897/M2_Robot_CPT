# M2 Human–Robot Interaction Controller

**Author:** Tiancheng (Gavin) Yang  
**Affiliation:** The University of Melbourne  
**License:** MIT

---

## Overview
This repository implements the control and data acquisition framework for the **M2 haptic robot** used in human–robot interaction (HRI) experiments. The system supports **deterministic perturbation scheduling**, **real-time force measurement**, and **automatic trial management** coordinated with Unity.

---

## Features
- Modular **state machine** (Standby, Calibration, ProbMove)
- **Deterministic trial sequencing** for reproducible probability conditions
- **Effort computation** from force-sensor data with baseline offset removal
- Real-time logging of:
  - Timestamp
  - Position / velocity
  - Commanded and sensed forces
  - Effort integral and total score
  - Session ID (from Unity)

---

## Data Logging Format
| Column | Description |
|---------|--------------|
| Timestamp | System time in seconds |
| PosX, PosY | End-effector position |
| VelX, VelY | End-effector velocity |
| FcmdX, FcmdY | Commanded force |
| FsX, FsY | Measured force |
| Effort | Accumulated user effort |
| Score | Current trial score |
| SID | Session ID provided by Unity |

---

## Usage
1. Launch the M2 machine binary (`M2Machine_APP`).
2. Wait for Unity connection.
3. Send `BGIN` command to enter **ProbMove** mode.
4. Configure parameters:
   - `S_SID`: Subject ID (integer, ≤ 64-bit)
   - `S_PB`: Probability (0.1 → 1 left, 9 up)
   - `S_MD`: Mode selection (v1 / v2)
5. Start the experiment with `STRT`.
6. Logged data will be written to `/logs/M2Machine_*.csv`.

---

## License
This project is licensed under the MIT License.  
© 2025 Tiancheng Yang, The University of Melbourne.