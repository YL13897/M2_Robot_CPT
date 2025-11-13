M2 Probabilistic Move Controller — Notices

Author
- Tiancheng Yang (University of Melbourne), 2025

License
- MIT License (see LICENSE). SPDX-License-Identifier: MIT

Data and Privacy
- The application writes CSV logs under `logs/` (e.g., `M2Machine.csv`, `M2ProbMove_<session>.csv`,
  `PreloadWindow_<session>.csv`, `TrialTags_<session>.csv`).
- Session identifiers may be personal or study-linked. Anonymize/redact session IDs and any
  other sensitive fields before sharing logs externally. Ensure compliance with your ethics
  approval and institutional data handling policies.

UI/Runtime Integration
- Unity UI communicates commands via the FLNLHelper service (e.g., `BGIN`, `STRT`, `HALT`).
- Preload detection is configurable at runtime:
  - `S_PLT <threshold_N>`: set preload force threshold in Newtons (default 3.0)
  - `S_PLW <window_s>`: set preload window in seconds (default 0.200)
- A `preload=0/1` tag is included in `TRIAL_BEGIN` messages for UI display.

Deterministic Scheduling
- LEFT/UP perturbation directions are arranged per block using a seeded shuffle to avoid strict
  alternation under 50/50 conditions.

Contact
- For questions about this implementation or experiments, please include your configuration
  (score mode, target successes, max trials, S_PLT/S_PLW) and relevant log snippets.
