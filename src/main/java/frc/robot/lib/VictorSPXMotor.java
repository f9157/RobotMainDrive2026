package frc.robot.lib;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class VictorSPXMotor implements Motor {
    VictorSPX motor;
    boolean inverted;
    int canId;
    String name;

    public VictorSPXMotor(int canId, String name, boolean inverted) {
        this.canId = canId;
        this.name = name;
        this.inverted = inverted;
        this.motor = new VictorSPX(canId);
        this.motor.setInverted(this.inverted);
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
        this.motor.set(VictorSPXControlMode.PercentOutput, ref);
    }

    public double getDutyOut() {
        return this.motor.getMotorOutputPercent();
    }

    public void follow(int otherCanId, boolean invert) {
        this.inverted = inverted;
        this.motor.setInverted(this.inverted);
        this.motor.set(VictorSPXControlMode.Follower, otherCanId);
    }
}
