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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.HoodPosition;

public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax        m_flywheelMotor;
    private final RelativeEncoder m_flywheelEncoder;
    private final DoubleSolenoid  m_hoodSolenoid;
    private final PIDController   m_flywheelPID;
    private final InterpolatingDoubleTreeMap m_flywheelTable = new InterpolatingDoubleTreeMap();

    private double       m_targetRPM      = 0;
    private HoodPosition m_hoodPosition   = HoodPosition.LOW;
    private boolean      m_shooterActive  = false;
    private double       m_rpmTrimPercent = 0;

    public ShooterSubsystem() {
        m_flywheelMotor = new SparkMax(ShooterConstants.kFlywheelCanId, MotorType.kBrushless);
        SparkMaxConfig flywheelCfg = new SparkMaxConfig();
        flywheelCfg.idleMode(IdleMode.kCoast).smartCurrentLimit(40);
        m_flywheelMotor.configure(flywheelCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_flywheelEncoder = m_flywheelMotor.getEncoder();

        m_hoodSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            ShooterConstants.kHoodSolenoidForwardChannel,
            ShooterConstants.kHoodSolenoidReverseChannel);
        m_hoodSolenoid.set(Value.kReverse);

        m_flywheelPID = new PIDController(ShooterConstants.kFlywheelP, 0, 0);
        m_flywheelPID.setTolerance(ShooterConstants.kFlywheelToleranceRPM);

        for (double[] row : ShooterConstants.kFlywheelRPMTable) m_flywheelTable.put(row[0], row[1]);
    }

    public void setDistanceTarget(double distanceMeters) {
        m_targetRPM    = m_flywheelTable.get(distanceMeters) * (1.0 + m_rpmTrimPercent / 100.0);
        m_hoodPosition = distanceMeters < ShooterConstants.kHoodSwitchDistanceMeters ? HoodPosition.LOW : HoodPosition.HIGH;
        m_shooterActive = true;
    }

    public void setHoodPosition(HoodPosition position) {
        m_hoodPosition = position;
        m_hoodSolenoid.set(position == HoodPosition.HIGH ? Value.kForward : Value.kReverse);
    }

    public void setRPMTrim(double trimPercent) { m_rpmTrimPercent = MathUtil.clamp(trimPercent, -10.0, 10.0); }

    public void setShooterActive(boolean active) {
        m_shooterActive = active;
        if (!active) { m_flywheelMotor.set(0); m_hoodSolenoid.set(Value.kOff); }
    }

    public boolean isReadyToShoot()       { return m_shooterActive && m_flywheelPID.atSetpoint(); }
    public double  getFlywheelRPM()       { return m_flywheelEncoder.getVelocity(); }
    public double  getTargetRPM()         { return m_targetRPM; }
    public HoodPosition getHoodPosition() { return m_hoodPosition; }

    public void stop() {
        m_flywheelMotor.set(0); m_hoodSolenoid.set(Value.kOff);
        m_shooterActive = false; m_rpmTrimPercent = 0;
    }

    @Override
    public void periodic() {
        if (m_shooterActive) {
            m_hoodSolenoid.set(m_hoodPosition == HoodPosition.HIGH ? Value.kForward : Value.kReverse);
            m_flywheelMotor.set(MathUtil.clamp(
                ShooterConstants.kFlywheelFF * m_targetRPM + ShooterConstants.kFlywheelP * (m_targetRPM - getFlywheelRPM()),
                0, 1));
        }
        SmartDashboard.putNumber("Shooter/FlywheelRPM",   getFlywheelRPM());
        SmartDashboard.putNumber("Shooter/TargetRPM",     m_targetRPM);
        SmartDashboard.putNumber("Shooter/RPMError",      m_targetRPM - getFlywheelRPM());
        SmartDashboard.putNumber("Shooter/RPMTrim%",      m_rpmTrimPercent);
        SmartDashboard.putString("Shooter/HoodPosition",  m_hoodPosition.name());
        SmartDashboard.putBoolean("Shooter/ReadyToShoot", isReadyToShoot());
        SmartDashboard.putBoolean("Shooter/Active",       m_shooterActive);
    }
}
