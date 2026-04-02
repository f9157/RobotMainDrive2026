# REBUILT 2026 — Robot Setup Guide

## Controller Layout

### Driver (port 0)
| Input | Action |
|---|---|
| Left stick | Translate (field-relative) |
| Right stick X | Rotate |
| Y | Zero gyro heading |
| X | Zero swerve module positions |

### Operator (port 1)
| Input | Action |
|---|---|
| Right Bumper (hold) | Auto-aim turret + spin up flywheel + set hood |
| Left Bumper (hold) | Ball feeder/indexer — add your feeder call in IO.java |
| Right stick X | Manual turret nudge |
| Left stick Y | Flywheel RPM fine-tune ±10% while shooting |
| A | Stow turret to 0° (forward) |
| B | Hood LOW preset (close/flat shot) |
| Y | Hood HIGH preset (far/steep shot) |

Watch Shooter/ReadyToShoot on SmartDashboard before pressing the feeder button.

---

## Camera Setup (PhotonVision)

### 1 — Flash the correct image
Flash the PhotonVision standalone image (not Raspberry Pi OS with PhotonVision installed on top).
Download from photonvision.org → Downloads → Raspberry Pi image. Flash with Balena Etcher.

### 2 — Connect and open the UI
Plug the Pi into your robot network switch. Open http://photonvision.local:5800 from a laptop on
the same network. If that fails try http://10.TE.AM.11:5800 with your team number.

### 3 — Camera settings
- Set the camera nickname to exactly: photonvision (must match VisionConstants.kCameraName)
- Resolution: 640x480 at 30fps
- Exposure: 5-10ms, manual (turn off auto-exposure)
- Pipeline type: AprilTag
- Tag family: 36h11
- Enable 3D Pose Estimation

### 4 — Calibrate the camera (do not skip)
1. Print the PhotonVision calibration checkerboard
2. Go to Camera Calibration in the UI
3. Take 25+ snapshots at varied angles and distances
4. Click Calibrate — target reprojection error below 1.0, ideally below 0.5
Without calibration pose estimates will be inaccurate, especially at range.

### 5 — Measure and enter camera position
Measure the camera lens position relative to the robot center and update VisionConstants.kCameraToRobot
in Constants.java. WPILib convention: X = forward, Y = left, Z = up. Negative pitch = tilted upward.
Even a 2-3 cm error shows up as aiming error on the field.

### 6 — Set a static IP
In PhotonVision settings assign the Pi a static IP of 10.TE.AM.11 so it is always reachable.

### 7 — Verify
Point the camera at a Hub AprilTag (IDs 2-5, 8-11, 18-21, 24-27). In the UI you should see the tag
outlined in green and a 3D pose readout on the right panel. If there is no 3D pose, redo calibration.

---

## Shooter Setup (GreyT Shooter 4"-6")

### CAN IDs to update in Constants.java
- kFlywheelCanId (ShooterConstants): currently 21
- kTurretCanId (TurretConstants): currently 20
- kHoodSolenoidForwardChannel / kHoodSolenoidReverseChannel: REV Pneumatic Hub channels 0 and 1
  If using a CTRE PCM change PneumaticsModuleType.REVPH to PneumaticsModuleType.CTREPCM in ShooterSubsystem.java

### Turret gear ratio
kGearRatio in TurretConstants is set to 10.0 (AndyMark 6.875in turret: 200T ring / 20T pinion = 10:1).
If you added an intermediate gearbox multiply: e.g. 5:1 gearbox = 50.0.

### Hood switch distance
kHoodSwitchDistanceMeters in ShooterConstants controls where the hood switches between LOW and HIGH.
Below this distance the actuator retracts (flat shot). Above it extends (steep shot).
Tune this value at the crossover distance on your real robot.

### Tuning the flywheel RPM table
The kFlywheelRPMTable values in ShooterConstants are placeholders. Tune them by shooting from known distances:
1. Park at exactly 1.5m from the Hub center (tape measure)
2. Press B to set hood LOW
3. Adjust flywheel RPM via SmartDashboard (Shooter/TargetRPM) until balls consistently fall in
4. Record that distance and RPM, update the table entry
5. Move to 2.0m, repeat
6. At kHoodSwitchDistanceMeters switch to hood HIGH (Y button) and re-tune RPM
7. Continue every 0.5m out to your maximum shooting range

### Tuning the turret PID
Start with kP = 0.025, kI = 0, kD = 0, kMaxOutput = 0.4.
Increase kP until the turret snaps to target quickly without oscillating.
Add kD = 0.001 if it overshoots. Leave kI at zero.

### Adding the ball feeder
In IO.java find the Left Bumper binding and replace the empty lambda with your feeder command:
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(/* your feeder command here */);
