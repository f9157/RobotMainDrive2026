package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final class IOConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorCOntrollerPort = 1;
    public static final double kDeadband = 0.08;
  }

  public static final class DriveConstants {
    // Robot geometry
    public static final double kTrackWidthMeters = 0.53;
    public static final double kWheelBaseMeters = 0.53;

    public static final Translation2d kFrontLeftLocation =
        new Translation2d(kWheelBaseMeters / 2.0,  kTrackWidthMeters / 2.0);
    public static final Translation2d kFrontRightLocation =
        new Translation2d(kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0);
    public static final Translation2d kBackLeftLocation =
        new Translation2d(-kWheelBaseMeters / 2.0,  kTrackWidthMeters / 2.0);
    public static final Translation2d kBackRightLocation =
        new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0);

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            kFrontLeftLocation,
            kFrontRightLocation,
            kBackLeftLocation,
            kBackRightLocation
        );

    // CAN IDs
    public static final int kFrontLeftDriveCanId = 2;
    public static final int kFrontLeftTurnCanId = 4;
    public static final int kFrontRightDriveCanId = 6;
    public static final int kFrontRightTurnCanId = 8;
    public static final int kBackLeftDriveCanId = 10;
    public static final int kBackLeftTurnCanId = 12;
    public static final int kBackRightDriveCanId = 14;
    public static final int kBackRightTurnCanId = 16;
    public static final int kPigeonIMU = 9;

    // Absolute encoders
    public static final int kFrontLeftAbsEncoderPort = 0;
    public static final int kFrontRightAbsEncoderPort = 1;
    public static final int kBackLeftAbsEncoderPort = 2;
    public static final int kBackRightAbsEncoderPort = 3;

    public static final double kFrontLeftAbsOffset = 0.00;
    public static final double kFrontRightAbsOffset = 0.688;
    public static final double kBackLeftAbsOffset = 0.238;
    public static final double kBackRightAbsOffset = 0.617;

    public static final boolean kFrontLeftDriveInverted = true;
    public static final boolean kFrontLeftTurnInverted = false;
    public static final boolean kFrontRightDriveInverted = false;
    public static final boolean kFrontRightTurnInverted = false;
    public static final boolean kBackLeftDriveInverted = true;
    public static final boolean kBackLeftTurnInverted = false;
    public static final boolean kBackRightDriveInverted = false;
    public static final boolean kBackRightTurnInverted = false;

    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeedRadPerSec = 2.0 * Math.PI;

    public static final double kWheelDiameterMeters = 0.1016; // 4 in
    public static final double kDriveMotorGearRatio = 6.75;
    public static final double kTurningMotorGearRatio = 22.8;
    public static final double kDriveLengthMeters = .66;

    public static final double kDrivePositionFactor =
        Math.PI * kWheelDiameterMeters / kDriveMotorGearRatio;
    public static final double kDriveVelocityFactor =
        kDrivePositionFactor / 60.0;

    public static final double kTurningPositionFactor =
        (2.0 * Math.PI) / kTurningMotorGearRatio;
    public static final double kTurningVelocityFactor =
        kTurningPositionFactor / 60.0;

    public static final double kTurningP = 4.0;
  }
}