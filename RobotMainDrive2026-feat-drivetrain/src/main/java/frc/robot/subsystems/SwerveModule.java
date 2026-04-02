package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;

  private final SparkClosedLoopController m_driveController;
  private final SparkClosedLoopController m_turnController;

  private final AnalogEncoder m_absoluteEncoder;
  private final double m_absoluteEncoderOffset;

  private final String name;

  private final PIDController m_turningPid = new PIDController(DriveConstants.kTurningP, 0.0, 0.0);

  public SwerveModule(
      int driveCanId,
      int turnCanId,
      boolean driveInverted,
      boolean turnInverted,
      int absoluteEncoderPort,
      double absoluteEncoderOffsetRad,
      String name) {

    this.name = name;
    m_driveMotor = new SparkMax(driveCanId, MotorType.kBrushless);
    m_turnMotor = new SparkMax(turnCanId, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = m_turnMotor.getEncoder();

    m_driveController = m_driveMotor.getClosedLoopController();
    m_turnController = m_turnMotor.getClosedLoopController();

    m_absoluteEncoder = new AnalogEncoder(absoluteEncoderPort);
    m_absoluteEncoderOffset = absoluteEncoderOffsetRad;

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.apply(new ClosedLoopConfig().pid(0.0001, 0, 0).feedbackSensor(FeedbackSensor.kPrimaryEncoder));
    driveConfig
        .inverted(driveInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);

    // driveConfig.encoder
    // .positionConversionFactor(DriveConstants.kDrivePositionFactor)
    // .velocityConversionFactor(DriveConstants.kDriveVelocityFactor);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.apply(new ClosedLoopConfig().pid(0.9, 0, 0).feedbackSensor(FeedbackSensor.kPrimaryEncoder));
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
    // turnConfig.encoder
    //     .positionConversionFactor(DriveConstants.kTurningPositionFactor)
    //     .velocityConversionFactor(DriveConstants.kTurningVelocityFactor);

    m_driveMotor.configure(
        driveConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turnMotor.configure(
        turnConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // m_turningPid.enableContinuousInput(-Math.PI, Math.PI);

    zero();
  }

  public double getTurnMotorPosition() {
    return m_turnEncoder.getPosition();
  }

  public double getDriveMotorPosition() {
    return m_driveEncoder.getPosition();
  }

  public void zero() {
    m_turnEncoder.setPosition(getAbsoluteEncoderRotation() * DriveConstants.kTurningMotorGearRatio);
  }

  public void setZero() {
    this.setRotation(Rotation2d.kZero);
  }

  public double getAbsoluteEncoderRaw() {
    return m_absoluteEncoder.get();
  }

  public double getAbsoluteEncoderRotation() {
    return this.getAbsoluteEncoderRaw() - this.m_absoluteEncoderOffset;
  }

  public double getDistance() {
    return (m_driveEncoder.getPosition() / DriveConstants.kDriveMotorGearRatio) * (Math.PI * DriveConstants.kWheelDiameterMeters);
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        this.getDistance(),
        this.getRotation2d());
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromRotations(m_turnEncoder.getPosition() / DriveConstants.kTurningMotorGearRatio);
  }

  public void setDesiredState(SwerveModuleState desiredState) {

    desiredState.optimize(getRotation2d());
    this.setRotation(desiredState.angle);
    this.setDriveVelocity(desiredState.speedMetersPerSecond);
  }

  private void setRotation(Rotation2d angle) {
    double turnOutput = angle.getRotations() * DriveConstants.kTurningMotorGearRatio;
    m_turnMotor.getClosedLoopController().setSetpoint(turnOutput, ControlType.kPosition);
  }

  private void setDriveVelocity(double metersPerSecond) {
    double driveRotationsPerSecond = metersPerSecond / (DriveConstants.kWheelDiameterMeters * Math.PI);

    double driveOutput = driveRotationsPerSecond
        * DriveConstants.kDriveMotorGearRatio * 60;
    m_driveMotor.getClosedLoopController().setSetpoint(driveOutput, ControlType.kVelocity);

  }

  public void stop() {
    m_driveMotor.stopMotor();
    m_turnMotor.stopMotor();
  }

  private String moduleDiagnosticKey(String key) {
    return "swerve/"+this.name+"/"+key;
  }

  private void putNumber(String key, double val) {
    SmartDashboard.putNumber(this.moduleDiagnosticKey(key), val);
  }

  public void sendDiagnostic() {
      this.putNumber("RawAbsoluteEnc", this.getAbsoluteEncoderRaw());
      this.putNumber("AbsoluteEncoder", this.getAbsoluteEncoderRotation());
      this.putNumber("Rotation", this.getRotation2d().getDegrees());
  }
}
