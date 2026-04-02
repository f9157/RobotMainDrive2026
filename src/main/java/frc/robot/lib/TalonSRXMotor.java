package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonSRXMotor implements Motor, MotorFollow {
    
    TalonSRX motor;
    boolean inverted;
    int canId;

    public TalonSRXMotor(int canId, boolean inverted) {
        this.canId = canId;
        this.inverted = inverted;
        this.motor = new TalonSRX(canId);
        this.motor.setInverted(this.inverted);
    }

    public String getMotorType() {
        return "TalonSRX";
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

    public void follow(int otherCanId, boolean invert) {
        this.inverted = inverted;
        this.motor.setInverted(this.inverted);
        this.motor.set(TalonSRXControlMode.Follower, otherCanId);
    }
}
