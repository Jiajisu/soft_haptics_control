# Soft Haptic Gripper (CHAI3D + Pneumatic Soft Actuator + MicronTracker)
**Real-time haptic rendering + pneumatic soft-actuator control + psychophysics user-study framework (adaptive staircase + auto logging).**

This project integrates:
- **CHAI3D** virtual environment & haptic loop (high-rate rendering)
- **Pneumatic soft continuum actuator** driven by **Arduino** (serial pressure commands)
- **MicronTracker (MT)** 6-DoF optical tracking (base-referenced calibration + rigid-body transforms)
- **Two psychophysics experiments** with robust session persistence and automatic CSV logging


### ✅ System-level engineering (end-to-end)
- Built a **full stack** from device I/O → real-time tracking → control → experiment logic → data logging.
- Demonstrates **robotics + haptics + real-time C++** integration (multi-threading, safety clamps, state machines).

### ✅ Real-time architecture
- **Haptics loop** runs continuously (CHAI3D), while a dedicated **MicronTracker thread** streams pose snapshots safely via mutex.
- Latency-tolerant design: missing MT frames fall back to the last valid snapshot briefly (prevents control spikes).

### ✅ Model-based soft actuator control
- Implements a **piecewise constant curvature (PCC)** kinematic model and an **analytical Jacobian**.
- Supports both:
  - **Open-loop** (pressure → predicted tip position)
  - **Closed-loop** correction (tracking feedback + slow loop fusion)

### ✅ Psychophysics built-in (ready for user studies)
- Experiment 1: **Stiffness discrimination** across **2 interaction modes × 3 directions × 2 polarities** (12 conditions)
- Experiment 2: **Angle/plane alignment** task with cue counting and break scheduling
- Uses **adaptive staircase (Kesten-style step schedule)**, **reversal-based stopping**, and **auto-resume sessions**

---

## Demo (recommended)
Add your own media here:



---

## System overview

### Runtime components (high-level)
```
┌──────────────────────────┐
│        CHAI3D World       │
│  tool ↔ virtual objects   │
└─────────────┬────────────┘
              │ force/interaction
              v
┌──────────────────────────┐      ┌──────────────────────────┐
│   ControlPCC (soft arm)  │<─────│  MicronTracker (MT)      │
│  PCC FK + Jacobian       │      │  base-referenced poses   │
│  open/closed-loop        │      │  dedicated MT thread     │
└─────────────┬────────────┘      └──────────────────────────┘
              │ pressure commands
              v
┌──────────────────────────┐
│ Arduino (SerialPort)     │
│ "p1,p2,p3\n"             │
│ pressure safety clamp     │
└──────────────────────────┘
              │
              v
┌──────────────────────────┐
│ UserStudyManager         │
│ Exp1 staircase + Exp2     │
│ session persistence + CSV │
└──────────────────────────┘
```

---

## Repository structure 

- **`Soft_gripper.cpp`**
  - Main entry point, CHAI3D world/UI setup
  - Haptics loop (`updateHaptics`)
  - MicronTracker loop (`MTLoop`) + snapshot sharing
  - Calibration workflow (base reference + rigid body + zeroing)
  - Experiment mode switching + keyboard controls

- **`ControlPCC.hpp/.cpp`**
  - Soft actuator model (pressure↔length mapping)
  - PCC forward kinematics + **analytical Jacobian**
  - Iterative motion update + closed-loop correction

- **`MicronTrackerWrapper.hpp`**
  - MT initialization, marker loading, pose/rotation matrix retrieval
  - Relative transforms (actuator vs finger), timestamping

- **`UserStudyManager.hpp/.cpp`**
  - Experiment 1: conditions, staircases, reversal stopping, persistence
  - Experiment 2: grouped sequences, timing, cue counting, breaks
  - Automatic CSV output formats

- **`SerialPort.hpp/.cpp`**
  - Windows serial wrapper (COM port)
  - Robust open/read/write helpers

- **`TrajectoryGenerator.hpp/.cpp`**
  - Circle/line/transition/sine trajectories for characterization/control testing

- **`TimeUtil.hpp/.cpp`**
  - High-resolution timing utilities

- **Calibration assets**
  - `rigid_body_calibration.txt`, `finger_T_bot.txt`, `actuator_T_top.txt`
  - MT marker templates: `base`, `finger`, `actuator`, `bot`, `top`

---

## Requirements

### OS / Toolchain
- **Windows** (SerialPort uses WinAPI)
- Visual Studio (recommended) or any MSVC-based build pipeline

### Libraries / SDKs
- **CHAI3D** + dependencies (OpenGL/GLFW)
- **MicronTracker SDK** (MTC headers/libs)
- (Optional/legacy) **Polhemus Viper** headers referenced by `frameTrans.*` (`VPcmdIF.h`)

> Note: This repo focuses on the C++ integration layer. You may need to configure include/lib paths for CHAI3D + MT SDK in your build system.

---

## Hardware assumptions (what the code expects)
- A **3-chamber pneumatic soft actuator** controlled by Arduino
- Arduino firmware expects **newline-terminated CSV string**:
  - `p1,p2,p3\n`
  - values are clamped in software (safety)
- MicronTracker markers named exactly:
  - `base`, `finger`, `actuator`, `bot`, `top`

---

## Quick start (how to run)

1. **Build**
   - Configure include/lib for CHAI3D + MicronTracker
   - Build the executable

2. **Connect devices**
   - MicronTracker camera connected + templates available
   - Arduino connected (default port is `COM3`)

3. **Run**
   - Launch executable
   - Follow console instructions for keyboard controls

---

## Keyboard controls (most important)

### General
- `Q` / `ESC`: Quit
- `M`: Toggle vertical mirroring
- `S`: Save shadowmap screenshot

### Serial / logging (characterization mode)
- `O`: Open CSV for characterization logging
- `W`: Toggle recording MT pose data into CSV
- `K`: Toggle writing scaled force into CSV

### Calibration (MicronTracker)
- `I`: Initialize **base reference frame** (requires `base` visible)
- `Z`: Enter/exit **rigid-body calibration** wizard
  - In calibration mode: `SPACE` records the current step
  - Sequence: **finger → bot → actuator → top**
- `V`: Load/save calibration + toggle visualization spheres
- `B`: Zero calibration (uses multiple samples; requires calibration valid)

### Experiment modes
- `E`: Toggle **Experiment 1** (stiffness discrimination)
- `R`: Toggle **Experiment 2** (angle/plane task)

#### Experiment 1 (stiffness discrimination)
- `T`: Start next trial
- `1`: Left stiffer
- `2`: Right stiffer
- Manual group shortcuts (for debugging / targeted runs):
  - `5/6/7`: Force-to-Position with X/Y/Z
  - `8/9/0`: Force-to-Force with X/Y/Z

#### Experiment 2 (angle/plane alignment)
- `T`: Start next trial
- `F`: Play force cue (counts usage)
- `←/→`: Adjust user angle
- `↑/↓`: Adjust plane spin (combined 3D mode)
- `SPACE`: Confirm/record trial result
- `Y` or `F6`: Toggle dev/debug overlay

---

## Calibration & coordinate frames 

### 1) Base reference (`I`)
Defines a stable world reference by capturing the `base` marker pose and applying a fixed offset rotation.
After this, all marker poses are transformed into the **base frame**.

### 2) Rigid-body calibration (`Z` + `SPACE`)
Records four marker poses in the base frame and computes:
- `finger_T_bot`
- `actuator_T_top`

These transforms can be saved/loaded (`V`) and are used for consistent rigid-body pose estimation.

### 3) Zero calibration (`B`)
Averages multiple samples to define a “neutral” pose baseline (e.g., to align actuator height with `h0`).

---

## ControlPCC (soft actuator model + controller)

### Pressure → chamber length mapping
- Piecewise model:
  - negative pressure uses a calibrated polynomial segment
  - positive pressure uses a calibrated linear segment
- Output is three chamber lengths used by PCC geometry.

### PCC forward kinematics
Computes tip position from three chamber lengths using PCC parameters:
- curvature κ, bending plane φ, and arc length θ-related terms

### Jacobian-based iterative update
Two main interfaces:
- `updateMotion(...)`: iterative update using model prediction
- `updateMotionCloseLoop(..., sensorPNO)`: closed-loop correction using tracked pose

### Safety
- Pressure commands are clamped in software (min/max)
- Additional logic prevents sending new commands if a channel hits safety threshold (holds last safe combination)

---

## Experiment 1: Stiffness discrimination (adaptive staircase)

### Conditions
**12 conditions total**:
- Interaction modes: **Force-to-Position** vs **Force-to-Force**
- Contact directions: **X / Y / Z**
- Surface polarity: **+ axis / − axis**
Each condition runs **two staircases**:
- `s0`: low-start comparison stiffness
- `s1`: high-start comparison stiffness

### Staircase logic (Kesten-style schedule)
- Step size decays with trial count:
  - `step = startStep / (1 + beta * (trialCount - 1))`
- Reversal counted when step direction flips
- Stop when reaching target reversal count (default: 6)

### Auto persistence (resume-friendly)
For each user ID, the program stores:
- `user_<id>_sequence.txt` (condition order)
- `user_<id>_progress.txt` (current state)
- `user_<id>_experiment_data.csv` (trial records)

---

## Experiment 1 CSV output (`user_<id>_experiment_data.csv`)
Columns:
- `TrialNumber`
- `ExperimentMode` (Force-to-Position / Force-to-Force)
- `Direction` (X/Y/Z)
- `SurfacePolarity` (+ / -)
- `ReferenceOnLeft` (0/1)
- `UserChoice` (0=Left, 1=Right)
- `CorrectAnswer` (0/1)
- `CompStifferYes` (0/1)
- `ReferenceStiffness`
- `ComparisonStiffness`
- `DeltaK` (Comparison - Reference)
- `StaircaseID` (0/1)
- `StaircaseDeltaK`
- `StepSize`
- `ReversalCount`
- `GroupIndex`

---

## Experiment 2: Angle / plane alignment

### Task
- System displays a plane/line configuration
- User adjusts angle (and optionally plane spin) to match a target
- Records:
  - target plane + target angle
  - user final plane + user final angle
  - completion time
  - force cue usage count

### Persistence & grouping
- Sequence file: `user_<id>_sequence_exp2.txt`
- CSV file: `user_<id>_experiment2_data.csv`
- Supports grouping by plane type and break scheduling

---

## Tips for adapting this repo for your setup

### Change Arduino port
In `Soft_gripper.cpp`, update:
- `const char* portNumber = "\\\\.\\COM3";`

### Add/modify experiment conditions
- Experiment 1 logic: `UserStudyManager` (conditions + staircase init)
- Rendering/contact surfaces: `Soft_gripper.cpp` (cube orientation/polarity + highlights)

### Swap tracking source
- Current mainline uses `MicronTrackerWrapper`
- Some utilities reference `PNODATA` (legacy Polhemus-friendly format). If needed, implement a new adapter that populates `PNODATA` from your tracker.

---

## Known limitations
- Windows-specific serial implementation
- Build requires local configuration of CHAI3D + MicronTracker SDK paths
- Arduino firmware is assumed (serial protocol documented above)

---

## Citation / Contact
If you use or extend this framework in academic work, please cite the associated papers

Maintainer: **Jiaji Su**  
Supervisor: **Zonghe Chua**
Lab: **ERIE Lab, Case Western Reserve University**
