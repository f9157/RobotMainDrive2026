package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants.DriveConstants;

public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;

  private final DutyCycleEncoder m_absoluteEncoder;
  private final double m_absoluteEncoderOffsetRad;

  private final PIDController m_turningPid =
      new PIDController(DriveConstants.kTurningP, 0.0, 0.0);

  public SwerveModule(
      int driveCanId,
      int turnCanId,
      boolean driveInverted,
      boolean turnInverted,
      int absoluteEncoderPort,
      double absoluteEncoderOffsetRad
  ) {
    m_driveMotor = new SparkMax(driveCanId, MotorType.kBrushless);
    m_turnMotor = new SparkMax(turnCanId, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = m_turnMotor.getEncoder();

    m_absoluteEncoder = new DutyCycleEncoder(absoluteEncoderPort);
    m_absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig
        .inverted(driveInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(50);
    driveConfig.encoder
        .positionConversionFactor(DriveConstants.kDrivePositionFactor)
        .velocityConversionFactor(DriveConstants.kDriveVelocityFactor);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(turnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);
    turnConfig.encoder
        .positionConversionFactor(DriveConstants.kTurningPositionFactor)
        .velocityConversionFactor(DriveConstants.kTurningVelocityFactor);

    m_driveMotor.configure(
        driveConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );
    m_turnMotor.configure(
        turnConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters
    );

    m_turningPid.enableContinuousInput(-Math.PI, Math.PI);

    resetToAbsolute();
  }

  public void resetToAbsolute() {
    m_turnEncoder.setPosition(getAbsoluteEncoderRadians());
  }

  public double getAbsoluteEncoderRadians() {
    double angle = m_absoluteEncoder.get() * 2.0 * Math.PI;
    angle -= m_absoluteEncoderOffsetRad;
    return MathUtil.angleModulus(angle);
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(MathUtil.angleModulus(m_turnEncoder.getPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState optimized =
        SwerveModuleState.optimize(desiredState, getRotation2d());

    double driveOutput =
        optimized.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond;

    double turnOutput = m_turningPid.calculate(
        getRotation2d().getRadians(),
        optimized.angle.getRadians()
    );

    m_driveMotor.set(driveOutput);
    m_turnMotor.set(turnOutput);
  }

  public void stop() {
    m_driveMotor.stopMotor();
    m_turnMotor.stopMotor();
  }
}