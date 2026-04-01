package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  Field2d field = new Field2d();

  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveCanId,
      DriveConstants.kFrontLeftTurnCanId,
      DriveConstants.kFrontLeftDriveInverted,
      DriveConstants.kFrontLeftTurnInverted,
      DriveConstants.kFrontLeftAbsEncoderPort,
      DriveConstants.kFrontLeftAbsOffset, "FL");

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveCanId,
      DriveConstants.kFrontRightTurnCanId,
      DriveConstants.kFrontRightDriveInverted,
      DriveConstants.kFrontRightTurnInverted,
      DriveConstants.kFrontRightAbsEncoderPort,
      DriveConstants.kFrontRightAbsOffset, "FR");

  private final SwerveModule m_backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveCanId,
      DriveConstants.kBackLeftTurnCanId,
      DriveConstants.kBackLeftDriveInverted,
      DriveConstants.kBackLeftTurnInverted,
      DriveConstants.kBackLeftAbsEncoderPort,
      DriveConstants.kBackLeftAbsOffset, "BL");

  private final SwerveModule m_backRight = new SwerveModule(
      DriveConstants.kBackRightDriveCanId,
      DriveConstants.kBackRightTurnCanId,
      DriveConstants.kBackRightDriveInverted,
      DriveConstants.kBackRightTurnInverted,
      DriveConstants.kBackRightAbsEncoderPort,
      DriveConstants.kBackRightAbsOffset, "BR");

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    DriveConstants.kFrontLeftLocation,
    DriveConstants.kFrontRightLocation,
    DriveConstants.kBackLeftLocation,
    DriveConstants.kBackRightLocation
  );

  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kPigeonIMU);


  public SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(kinematics,
  getGyroHeading(), this.getModulePositions(),
   Pose2d.kZero
  );


  public DriveSubsystem() {
    zeroHeading();
  }

  public SwerveModulePosition[] getModulePositions() {
      SwerveModulePosition[] l = {
        m_frontLeft.getModulePosition(),
        m_frontRight.getModulePosition(),
        m_backLeft.getModulePosition(),
        m_backRight.getModulePosition()
      };
      return l;
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    ChassisSpeeds chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed,
            ySpeed,
            rot,
            getHeading())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  public Rotation2d getGyroHeading() {
    return m_gyro.getRotation2d();
  }

  public Rotation2d getHeading() {
    return this.getPose2d().getRotation();
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

  public Pose2d getPose2d() {
    return odometry.getEstimatedPosition();
  }

  public double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, Constants.IOConstants.kDeadband);
  }

  public void setZero() {
    this.m_backLeft.setZero();
    this.m_backRight.setZero();
    this.m_frontLeft.setZero();
    this.m_frontRight.setZero();
  }

  @Override
  public void periodic() {

    this.odometry.update(this.getGyroHeading(), this.getModulePositions());

    field.setRobotPose(this.getPose2d());
    SmartDashboard.putData("field", field);
    m_backLeft.sendDiagnostic();
    m_backRight.sendDiagnostic();
    m_frontLeft.sendDiagnostic();
    m_frontRight.sendDiagnostic();
  }
}
