package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {

    private final SparkMax        m_turretMotor;
    private final RelativeEncoder m_encoder;
    private final PIDController   m_pid;

    private double  m_targetAngleDeg  = 0.0;
    private boolean m_autoAimEnabled  = false;

    public TurretSubsystem() {
        m_turretMotor = new SparkMax(TurretConstants.kTurretCanId, MotorType.kBrushless);

        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(IdleMode.kBrake)
           .smartCurrentLimit(TurretConstants.kCurrentLimitAmps)
           .inverted(TurretConstants.kTurretInverted);
        cfg.encoder
           .positionConversionFactor(TurretConstants.kPositionConversionFactor)
           .velocityConversionFactor(TurretConstants.kPositionConversionFactor / 60.0);

        m_turretMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = m_turretMotor.getEncoder();
        m_encoder.setPosition(0.0);

        m_pid = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        m_pid.setTolerance(TurretConstants.kToleranceDeg);
        m_pid.enableContinuousInput(-180.0, 180.0);
    }

    public void setAutoAim(boolean enabled) { m_autoAimEnabled = enabled; }
    public boolean isAutoAimEnabled()       { return m_autoAimEnabled; }
    public boolean isOnTarget()             { return m_pid.atSetpoint(); }
    public double  getTurretAngleDeg()      { return m_encoder.getPosition(); }

    public void updateAimAngle(Pose2d robotPose, Translation2d targetPos) {
        double dx = targetPos.getX() - robotPose.getX();
        double dy = targetPos.getY() - robotPose.getY();
        double fieldAngleDeg  = Math.toDegrees(Math.atan2(dy, dx));
        double robotYawDeg    = robotPose.getRotation().getDegrees();
        m_targetAngleDeg = MathUtil.inputModulus(fieldAngleDeg - robotYawDeg, -180.0, 180.0);
    }

    public void setManualAngle(double degrees) {
        m_targetAngleDeg = MathUtil.inputModulus(degrees, -180.0, 180.0);
    }

    public double getDistanceToTarget(Pose2d robotPose, Translation2d targetPos) {
        double dx = targetPos.getX() - robotPose.getX();
        double dy = targetPos.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    public void stop() { m_turretMotor.set(0.0); }

    @Override
    public void periodic() {
        if (m_autoAimEnabled) {
            double currentAngle = getTurretAngleDeg();
            double output = MathUtil.clamp(
                m_pid.calculate(currentAngle, m_targetAngleDeg),
                -TurretConstants.kMaxOutput, TurretConstants.kMaxOutput);
            if ((currentAngle >= TurretConstants.kMaxAngleDeg && output > 0) ||
                (currentAngle <= TurretConstants.kMinAngleDeg && output < 0)) {
                output = 0.0;
            }
            m_turretMotor.set(output);
        }

        SmartDashboard.putNumber("Turret/CurrentAngle_deg", getTurretAngleDeg());
        SmartDashboard.putNumber("Turret/TargetAngle_deg",  m_targetAngleDeg);
        SmartDashboard.putNumber("Turret/Error_deg",        m_pid.getError());
        SmartDashboard.putBoolean("Turret/OnTarget",        isOnTarget());
        SmartDashboard.putBoolean("Turret/AutoAimEnabled",  m_autoAimEnabled);
    }
}
