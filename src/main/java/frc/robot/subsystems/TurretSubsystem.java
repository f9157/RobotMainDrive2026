package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.TalonFXSMotor;

public class TurretSubsystem extends SubsystemBase {

    // private final SparkMax        m_turretMotor;
    // private final RelativeEncoder m_encoder;
    // private final PIDController   m_pid;

    private TalonFXSMotor turretMotor;

    private double  m_targetAngleDeg  = 0.0;
    // private boolean m_autoAimEnabled  = false;

    public TurretSubsystem() {
        this.turretMotor = new TalonFXSMotor(Constants.TurretConstants.kTurretCanId, "Turret");
    }

    private double motorToTurret(double motorValue) {
        return motorValue / Constants.TurretConstants.kTurretGearRatio;
    }

    private double turretToMotor(double turretValue) {
        return turretValue * Constants.TurretConstants.kTurretGearRatio;
    }

    // public void updateAimAngle(Pose2d robotPose, Translation2d targetPos) {
    //     double dx = targetPos.getX() - robotPose.getX();
    //     double dy = targetPos.getY() - robotPose.getY();
    //     double fieldAngleDeg  = Math.toDegrees(Math.atan2(dy, dx));
    //     double robotYawDeg    = robotPose.getRotation().getDegrees();
    //     m_targetAngleDeg = MathUtil.inputModulus(fieldAngleDeg - robotYawDeg, -180.0, 180.0);
    // }

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

    // public double getDistanceToTarget(Pose2d robotPose, Translation2d targetPos) {
    //     double dx = targetPos.getX() - robotPose.getX();
    //     double dy = targetPos.getY() - robotPose.getY();
    //     return Math.hypot(dx, dy);
    // }

    // public void stop() { m_turretMotor.set(0.0); }

    @Override
    public void periodic() {

        this.turretMotor.postMotorDiagnostics();
        SmartDashboard.putNumber("Turret/CurrentAngle_deg", getAngleDegrees());
        SmartDashboard.putNumber("Turret/TargetAngle_deg",  m_targetAngleDeg);
        // SmartDashboard.putBoolean("Turret/OnTarget",        isOnTarget());
        // SmartDashboard.putBoolean("Turret/AutoAimEnabled",  m_autoAimEnabled);
    }
}
