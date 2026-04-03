package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonSRXMotor implements Motor, MotorFollow {
    
    TalonSRX motor;
    boolean inverted;
    int canId;
    String name;

    public TalonSRXMotor(int canId, String name, boolean inverted) {
        this.canId = canId;
        this.name = name;
        this.inverted = inverted;
        this.motor = new TalonSRX(canId);
        this.motor.setInverted(this.inverted);
    }

    public TalonSRX getInner() {
        return this.motor;
    }

    public String getMotorType() {
        return "TalonSRX";
    }

    public String getMotorName() {
        return this.name;
    }

    public int getDeviceId() {
        return this.canId;
    }

    public void stop() {
        this.setDutyOut(0);
    }


    public void setDutyOut(double ref) {
        this.motor.set(TalonSRXControlMode.PercentOutput, ref);
    }

    public double getDutyOut() {
        return this.motor.getMotorOutputPercent();
    }

    public void setPosition(double ref) {
        this.motor.set(TalonSRXControlMode.Position, ref);
    }

    public void follow(int otherCanId, boolean invert) {
        this.inverted = inverted;
        this.motor.setInverted(this.inverted);
        this.motor.set(TalonSRXControlMode.Follower, otherCanId);
    }
}
