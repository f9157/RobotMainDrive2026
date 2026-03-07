package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveCanId,
          DriveConstants.kFrontLeftTurnCanId,
          DriveConstants.kFrontLeftDriveInverted,
          DriveConstants.kFrontLeftTurnInverted,
          DriveConstants.kFrontLeftAbsEncoderPort,
          DriveConstants.kFrontLeftAbsOffsetRad
      );

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveCanId,
          DriveConstants.kFrontRightTurnCanId,
          DriveConstants.kFrontRightDriveInverted,
          DriveConstants.kFrontRightTurnInverted,
          DriveConstants.kFrontRightAbsEncoderPort,
          DriveConstants.kFrontRightAbsOffsetRad
      );

  private final SwerveModule m_backLeft =
      new SwerveModule(
          DriveConstants.kBackLeftDriveCanId,
          DriveConstants.kBackLeftTurnCanId,
          DriveConstants.kBackLeftDriveInverted,
          DriveConstants.kBackLeftTurnInverted,
          DriveConstants.kBackLeftAbsEncoderPort,
          DriveConstants.kBackLeftAbsOffsetRad
      );

  private final SwerveModule m_backRight =
      new SwerveModule(
          DriveConstants.kBackRightDriveCanId,
          DriveConstants.kBackRightTurnCanId,
          DriveConstants.kBackRightDriveInverted,
          DriveConstants.kBackRightTurnInverted,
          DriveConstants.kBackRightAbsEncoderPort,
          DriveConstants.kBackRightAbsOffsetRad
      );

  private final Pigeon2 m_gyro = new Pigeon2(9);

  public DriveSubsystem() {
    zeroHeading();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rot,
                getHeading()
              )
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    SwerveModuleState[] states =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        DriveConstants.kMaxSpeedMetersPerSecond
    );

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, Constants.OIConstants.kDeadband);
  }
}