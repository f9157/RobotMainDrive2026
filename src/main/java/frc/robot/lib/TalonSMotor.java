package frc.robot.lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonSMotor implements PidMotor, MotorDiag {
    
    private int canId;

    private TalonFXS motor;


    private StatusSignal<AngularVelocity> velocity;

    private StatusSignal<Angle> position;

    private StatusSignal<Current> current;

    private MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

    private MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);

    public TalonSMotor(int canId) {
        this.canId = canId;
        this.motor = new TalonFXS(canId);
        this.position = this.motor.getPosition();
        this.velocity = this.motor.getVelocity();
        this.current = this.motor.getStatorCurrent();
    }

    public TalonFXS getInner() {
        return this.motor;
    }

    public void setDutyOut(double ref) {
        this.motor.set(ref);
    }

    public double getDutyOut() {
        return this.motor.get();
    }

    public void setPosition(double ref) {
        this.positionControl.Position = ref;
        this.motor.setControl(this.positionControl);
    }

    public double getPosition() {
        this.position.refresh();
        return this.position.getValueAsDouble();
    }

    public void setVelocity(double ref) {
        this.velocityControl.Velocity = ref;
        this.motor.setControl(this.velocityControl);
    }

    public double getVelocity() {
        this.velocity.refresh();
        return this.velocity.getValueAsDouble();
    }

    public double getCurrent() {
        this.current.refresh();
        return this.getCurrent();
    }

    public String getMotorType() {
        return "TalonFXS";
    }

    public int getDeviceId() {
        return this.canId;
    }


}
