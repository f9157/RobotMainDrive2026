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
| Left Bumper (hold) | Ball feeder/indexer — add your feeder call in `IO.java` |
| Right stick X | Manual turret nudge |
| Left stick Y | Flywheel RPM fine-tune ±10% while shooting |
| A | Stow turret to 0° (forward) |
| B | Hood LOW preset (close/flat shot) |
| Y | Hood HIGH preset (far/steep shot) |

Watch `Shooter/ReadyToShoot` on SmartDashboard before pressing the feeder button.

---

## Camera Setup (PhotonVision)

### 1 — Flash the correct image
Flash the PhotonVision standalone image (not Raspberry Pi OS + PhotonVision on top).
Download from photonvision.org → Downloads → Raspberry Pi image. Flash with Balena Etcher.

### 2 — Connect and open the UI
Plug the Pi into your robot network switch. Open `http://photonvision.local:5800` from a laptop on the same network. If that fails, try `http://10.TE.AM.11:5800`.

### 3 — Camera settings
- Set camera nickname to exactly `photonvision` (must match `VisionConstants.kCameraName`)
- Resolution: 640×480 at 30fps is a good starting point
- Exposure: 5–10ms manual (turn off auto-exposure)
- Pipeline type: **AprilTag**
- Tag family: **36h11**
- Enable **3D Pose Estimation**

### 4 — Calibrate the camera (do not skip)
1. Print the PhotonVision calibration checkerboard
2. Go to Camera Calibration in the UI
3. Take 25+ snapshots at varied angles and distances
4. Click Calibrate — target reprojection error below 1.0, ideally below 0.5

Without calibration, pose estimates will be inaccurate especially at range.

### 5 — Measure and enter camera position
Measure the camera lens position relative to the robot center and update `VisionConstants.kCameraToRobot` in `Constants.java`:

```java
public static final Transform3d kCameraToRobot = new Transform3d(
    new Translation3d(X, Y, Z),       // meters: X=forward, Y=left, Z=up
    new Rotation3d(0, Math.toRadians(PITCH), 0)  // negative pitch = tilted upward
);
```

Even a 2–3 cm error here shows up as aiming error on field. Measure carefully.

### 6 — Set a static IP
In PhotonVision settings, assign the Pi a static IP of `10.TE.AM.11` so it is always reachable. Without this it may get a new DHCP address and fail to connect mid-match.

### 7 — Verify
Point the camera at a Hub AprilTag (IDs 2–5, 8–11, 18–21, 24–27). In the UI you should see the tag outlined in green and a 3D pose readout on the right. If there is no 3D pose, redo calibration.

---

## Shooter Setup (GreyT Shooter 4"–6")

### Constants to update in `Constants.java`

**CAN IDs — `ShooterConstants`**
```java
public static final int kFlywheelCanId              = 21;  // update to your ID
public static final int kHoodSolenoidForwardChannel = 0;   // REV PH channel
public static final int kHoodSolenoidReverseChannel = 1;   // REV PH channel
```
If using a CTRE PCM instead of a REV Pneumatic Hub, change `PneumaticsModuleType.REVPH` to `PneumaticsModuleType.CTREPCM` in `ShooterSubsystem.java`.

**Hood switch distance — `ShooterConstants`**
```java
public static final double kHoodSwitchDistanceMeters = 3.0;
```
Below this distance the hood retracts (LOW/flat). Above it extends (HIGH/steep). Tune this at the crossover distance on your real robot.

**Turret CAN ID and gear ratio — `TurretConstants`**
```java
public static final int    kTurretCanId  = 20;   // update to your ID
public static final double kGearRatio    = 10.0; // AndyMark 6.875in turret = 10:1
```
If you added an intermediate gearbox between the NEO and the pinion, multiply: e.g. a 5:1 gearbox makes this `50.0`.

### Tuning the flywheel RPM table
The `kFlywheelRPMTable` values in `ShooterConstants` are placeholders. Tune them by shooting from known distances:

1. Park the robot at exactly **1.5 m** from the Hub center (tape measure)
2. Set hood to LOW (B button on operator controller)
3. Adjust flywheel RPM via SmartDashboard → `Shooter/TargetRPM` until balls consistently fall in
4. Record that distance and RPM, update the table
5. Move to **2.0 m**, repeat
6. At `kHoodSwitchDistanceMeters`, switch hood to HIGH (Y button) and re-tune RPM
7. Continue every 0.5 m out to your maximum shooting range

The more data points you add, the smoother the interpolation. Aim for one entry every 0.5 m.

### Tuning the turret PID
Start with only `kP = 0.025`, `kI = 0`, `kD = 0`, and `kMaxOutput = 0.4`.
Increase `kP` until the turret snaps to the target quickly without oscillating.
Add a small `kD` (try `0.001`) if it overshoots. Leave `kI` at zero.

### Adding the ball feeder
In `IO.java`, replace the empty lambda in the Left Bumper binding with your feeder subsystem call:
```java
new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
    .whileTrue(/* your feeder command here */);
```
