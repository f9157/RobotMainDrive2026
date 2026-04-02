package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final class IOConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorCOntrollerPort = 1;
    public static final double kDeadband = 0.08;
  }

  public static final class PositionConstants {
    public static final Pose2d BlueHubPose = new Pose2d(
        Units.inchesToMeters(181.56),
        Units.inchesToMeters(158.32),
        Rotation2d.kZero);
    public static final Pose2d RedHubPose = new Pose2d(
        Units.inchesToMeters(468.56),
        Units.inchesToMeters(158.32),
        Rotation2d.kZero);

    public static final double HubZ = Units.inchesToMeters(120.36);
    public static final double RobotZ = Units.inchesToMeters(15.0);
    public static final double DeltaZ = HubZ - RobotZ;

  }

  public class TargetingConstants {
      public static final double blueAllianceZoneMaxXMeters = Units.inchesToMeters(158.6);
      public static final double redAllianceZoneMinXMeters = Units.inchesToMeters(492.6);

      public static final double gravity = 9.81;
      // used to determine the flywheel speed based on robot distance from the hub
      public static final double shotSlope = 5.662;
      public static final double RPSAtZeroDistance = 33.86;

      // used to try to get the shot into the center of the hub
      public static final double fuelRadius = 0.07;
      public static final double goalRadius = 0.5;


      public static class FudgeConstants {
          // Each D-pad press changes the software's effective shot distance by this much
          public static final double shotDistanceFudgeStepMeters = 0.5;
      }

      public static InterpolatingDoubleTreeMap flywheelSpeedLookup = new InterpolatingDoubleTreeMap();
      public static void flyWheelSpeedLookupInitializer() {
          flywheelSpeedLookup.put(0.0, 33.86);
          flywheelSpeedLookup.put(4.0, 56.508);
          flywheelSpeedLookup.put(4.5, 62.66);
          flywheelSpeedLookup.put(5.0, 65.86);
      }
  }

  public static final class AutoConstants {
      public static final double cornerOffsetY = 1.5;
      public static final double cornerOffsetX = 3;
      // for offsets, x positive is backwards from the apriltag, y positive is to the
      // right of the apriltag
      // so for example, and x of 1 and a y of 1 would be 1 meter back and 1 meter to
      // the right of the apriltag


      public static final Pose2d blueAllianceTop = new Pose2d(0 + cornerOffsetX, 8.069 - cornerOffsetY, Rotation2d.kZero);//(0,8.069)
      public static final Pose2d blueAllianceBottom = new Pose2d(0 + cornerOffsetX, 0 + cornerOffsetY, Rotation2d.kZero);
      public static final Pose2d redAllianceTop = new Pose2d(16.54 - cornerOffsetX, 8.069 - cornerOffsetY, Rotation2d.kZero); //(8.069, 16.54)
      public static final Pose2d redAllianceBottom = new Pose2d(16.54 - cornerOffsetX,0 + cornerOffsetY, Rotation2d.kZero);  //(16.54, 0)
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

  public static final class VisionConstants {

    public static final double kFlatStdDevXY = 0.2;

    public static final double kMaxFlatDistanceMeters = 2;

    public static final double kExpMultiplier = 1;

    public static final Transform3d kLeftCameraOffset = Transform3d.kZero;

    public static final Transform3d kRightCameraOffset = Transform3d.kZero;
  }

    public static class StaticPID {
        double kP;
        double kI;
        double kD;
        double kV;
        double kS;
        double maxAccel;
        double maxVel;

        public StaticPID(double kP, double kI, double kD, double kV, double kS, double maxAccel, double maxVel) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kV = kV;
            this.kS = kS;
            this.maxAccel = maxAccel;
            this.maxVel = maxVel;
        }

        public MotionMagicConfigs applyTalon(SlotConfigs slot) {
            slot.kV = this.kV;
            slot.kP = this.kP;
            slot.kI = this.kI;
            slot.kD = this.kD;
            slot.kS = this.kS;
            return new MotionMagicConfigs().withMotionMagicAcceleration(maxAccel).withMotionMagicCruiseVelocity(maxVel);
        }

        public void applySparkMax(SparkMax controller) {

        }

    }

    public static final class FlywheelConstants {
        public static final int kLeftMainFlywheelCanId = 32;

        public static final int kRightFollowFlywheelCanId = 34;

        public static final StaticPID FlywheelPID = new StaticPID(0, 0, 0, 0.13, 0.2, 200, 100);
    }

    public static final class IntakeConstants {
        public static final int kDeployMotorCanId = 20;

        public static final StaticPID DeployPID = new StaticPID(0,0,0,0.13,0.4,30,30);

        public static final int kWheelMainMotorCanId = 21;
        public static final int kWheelFollowMotorCanId = 22;

        public static final StaticPID WheelPID = new StaticPID(0,0,0,0,0,0,0);

        public static final double kIntakeRetractPosition = 0;

        public static final double kIntakeDeployPosition = 0;

        public static final double kIntakeVelocity = 10;
    }

    public static final class IndexerConstants {
        public static final int kMotorCanId = 45;
        public static final double kIndexerSpeed = 0.9;
    }

    public static final class TurretConstants {
        public static final int kTurretCanId = 51;
        public static final double kTurretGearRatio = 10; // Double Check this
        public static final StaticPID turretPID = new StaticPID(0,0,0,0.13,0,0,0);
    }
}
