package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.lib.TalonFXSMotor;
import frc.robot.lib.TalonFXSMotor.MotorType;

public class TurretSubsystem extends SubsystemBase {

    // private final SparkMax        m_turretMotor;
    // private final RelativeEncoder m_encoder;
    // private final PIDController   m_pid;

    private TalonFXSMotor turretMotor;

    private double  m_targetAngleDeg  = 0.0;
    // private boolean m_autoAimEnabled  = false;

    public TurretSubsystem() {
        this.turretMotor = new TalonFXSMotor(Constants.TurretConstants.kTurretCanId, "Turret", MotorType.getNeo());

        this.turretMotor.apply(TurretConstants.turretPID);
    }

    private double motorToTurret(double motorValue) {
        return motorValue / Constants.TurretConstants.kTurretGearRatio;
    }

    private double turretToMotor(double turretValue) {
        return turretValue * Constants.TurretConstants.kTurretGearRatio;
    }

    public void setAngleDegrees(double degrees) {
        this.m_targetAngleDeg = MathUtil.inputModulus(degrees, -180.0, 180.0);
        double turretRotation = this.m_targetAngleDeg / 360;
        double motorRotation = this.turretToMotor(turretRotation);
        this.turretMotor.setPosition(motorRotation);
    }

    public double getAngleDegrees() {
        double motorRotation = this.turretMotor.getPosition();
        double turretRotation = this.motorToTurret(motorRotation);
        return turretRotation * 360;
    }

    public boolean atTargetAngle(double tolerance) {
        return Math.abs(this.getAngleDegrees() - this.m_targetAngleDeg) < tolerance;
    }


    @Override
    public void periodic() {

        this.turretMotor.postMotorDiagnostics();
        SmartDashboard.putNumber("Turret/CurrentAngle_deg", getAngleDegrees());
        SmartDashboard.putNumber("Turret/TargetAngle_deg",  m_targetAngleDeg);
    }
}
