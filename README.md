## 2026-Rebuilt

CTEC Robotics Program code for 2026 — v1.1.0

### Overview
The 2026 robot features a rear intake, a mid-robot hopper/conveyor, and a front shooter with an articulating hood. Shooting is automated based on robot pose (polar targeting) and uses multiple Limelights for pose estimation.

### Driver Controls
| Action                              | Control                         |
| ----------------------------------- | ------------------------------- |
| Movement                            | Left analog stick               |
| Rotation                            | Right analog stick (horizontal) |
| Shoot                               | Right trigger (RT)              |
| Intake                              | Left trigger (LT)               |
| Reset Field Position (Front Camera) | D-pad Down                      |
| Toggle Polar/Cartesian Mode         | Left bumper (LB)                |
| Slowdown Drive (Cartesian)          | Right bumper (RB) — hold        |
| Force Stow Hood                     | A button                        |
| Shoot While Against Hub             | X button                        |
| Cancel All Active Commands          | Y Button                        |

### Key Features
- 4 Neo Vortex motors on the main shooter flywheel (high-speed flywheel).
- 2 Talon FX (Kraken X60) motors for the shooter feeder.
- Neo Vortex motor on the conveyor (direction reversed in code).
- Intake driven by a Talon FX (Kraken X60).
- Articulating hood for shot angle adjustments.
- 3 Limelights used for pose estimation and targeting.
- Automatic shooting based on robot location + vision.