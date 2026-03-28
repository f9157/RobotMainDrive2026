package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.08;
  }

  public static final class DriveConstants {
    public static final double kTrackWidthMeters = 0.53;
    public static final double kWheelBaseMeters  = 0.53;

    public static final Translation2d kFrontLeftLocation  = new Translation2d( kWheelBaseMeters / 2.0,  kTrackWidthMeters / 2.0);
    public static final Translation2d kFrontRightLocation = new Translation2d( kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0);
    public static final Translation2d kBackLeftLocation   = new Translation2d(-kWheelBaseMeters / 2.0,  kTrackWidthMeters / 2.0);
    public static final Translation2d kBackRightLocation  = new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);

    public static final int kFrontLeftDriveCanId  = 2;
    public static final int kFrontLeftTurnCanId   = 4;
    public static final int kFrontRightDriveCanId = 6;
    public static final int kFrontRightTurnCanId  = 8;
    public static final int kBackLeftDriveCanId   = 10;
    public static final int kBackLeftTurnCanId    = 12;
    public static final int kBackRightDriveCanId  = 14;
    public static final int kBackRightTurnCanId   = 16;
    public static final int kPigeonIMU            = 9;

    public static final int kFrontLeftAbsEncoderPort  = 0;
    public static final int kFrontRightAbsEncoderPort = 1;
    public static final int kBackLeftAbsEncoderPort   = 2;
    public static final int kBackRightAbsEncoderPort  = 3;

    public static final double kFrontLeftAbsOffset  = 0.00;
    public static final double kFrontRightAbsOffset = 0.688;
    public static final double kBackLeftAbsOffset   = 0.238;
    public static final double kBackRightAbsOffset  = 0.617;

    public static final boolean kFrontLeftDriveInverted  = true;
    public static final boolean kFrontLeftTurnInverted   = false;
    public static final boolean kFrontRightDriveInverted = false;
    public static final boolean kFrontRightTurnInverted  = false;
    public static final boolean kBackLeftDriveInverted   = true;
    public static final boolean kBackLeftTurnInverted    = false;
    public static final boolean kBackRightDriveInverted  = false;
    public static final boolean kBackRightTurnInverted   = false;

    public static final double kMaxSpeedMetersPerSecond  = 4.5;
    public static final double kMaxAngularSpeedRadPerSec = 2.0 * Math.PI;

    public static final double kWheelDiameterMeters   = 0.1016;
    public static final double kDriveMotorGearRatio   = 6.75;
    public static final double kTurningMotorGearRatio = 22.8;
    public static final double kDriveLengthMeters     = 0.66;

    public static final double kDrivePositionFactor   = Math.PI * kWheelDiameterMeters / kDriveMotorGearRatio;
    public static final double kDriveVelocityFactor   = kDrivePositionFactor / 60.0;
    public static final double kTurningPositionFactor = (2.0 * Math.PI) / kTurningMotorGearRatio;
    public static final double kTurningVelocityFactor = kTurningPositionFactor / 60.0;

    public static final double kTurningP = 4.0;
  }

  public static final class TurretConstants {
    public static final int     kTurretCanId            = 20;
    public static final boolean kTurretInverted         = false;
    public static final int     kCurrentLimitAmps       = 30;
    public static final double  kGearRatio              = 10.0;
    public static final double  kPositionConversionFactor = 360.0 / kGearRatio;
    public static final double  kP                      = 0.035;
    public static final double  kI                      = 0.0;
    public static final double  kD                      = 0.001;
    public static final double  kMaxOutput              = 0.6;
    public static final double  kToleranceDeg           = 1.5;
    public static final double  kMaxAngleDeg            =  180.0;
    public static final double  kMinAngleDeg            = -180.0;
  }

  public static final class VisionConstants {
    public static final String kCameraName = "photonvision";

    public static final edu.wpi.first.math.geometry.Transform3d kCameraToRobot =
        new edu.wpi.first.math.geometry.Transform3d(
            new edu.wpi.first.math.geometry.Translation3d(0.30, 0.0, 0.25),
            new edu.wpi.first.math.geometry.Rotation3d(0, Math.toRadians(-10), 0)
        );

    public static final double kMaxAmbiguity               = 0.2;
    public static final double kMaxSingleTagDistanceMeters = 4.0;

    public static final java.util.Set<Integer> kHubTagIds = java.util.Set.of(
        2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27);

    public static java.util.Set<Integer> getValidTagIds() {
      return kHubTagIds;
    }
  }

  public static final class ShooterConstants {
    public static final int kFlywheelCanId              = 21;
    public static final int kHoodSolenoidForwardChannel = 0;
    public static final int kHoodSolenoidReverseChannel = 1;

    public static final double kFlywheelP            = 0.0003;
    public static final double kFlywheelFF           = 0.00019;
    public static final double kFlywheelToleranceRPM = 100;

    public enum HoodPosition { LOW, HIGH }

    public static final double kHoodSwitchDistanceMeters = 3.0;

    public static final double[][] kFlywheelRPMTable = {
        { 1.5, 2200 },
        { 2.0, 2600 },
        { 2.5, 2950 },
        { 3.0, 3250 },
        { 3.5, 3500 },
        { 4.0, 3750 },
        { 4.5, 3950 },
        { 5.0, 4200 },
    };
  }

  public static final class FieldConstants {
    public static final edu.wpi.first.apriltag.AprilTagFieldLayout kFieldLayout =
        edu.wpi.first.apriltag.AprilTagFieldLayout.loadField(
            edu.wpi.first.apriltag.AprilTagFields.k2026Reefscape);

    public static final Translation2d kHubPosition =
        kFieldLayout.getTagPose(9).get().getTranslation().toTranslation2d();

    public static Translation2d getHubPosition() {
      return kHubPosition;
    }
  }
}
