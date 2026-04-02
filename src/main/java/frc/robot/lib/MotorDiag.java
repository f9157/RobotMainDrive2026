package frc.robot.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface MotorDiag extends PidMotor {

    private String motorKey() {
        return this.getMotorType()+"_"+this.getMotorName()+"_"+this.getDeviceId();
    }

    default void postMotorDiagnostics() {
        SmartDashboard.putNumber(this.motorKey()+"/position", this.getPosition());
        SmartDashboard.putNumber(this.motorKey()+"/velocity", this.getVelocity());
        SmartDashboard.putNumber(this.motorKey()+"/current", this.getCurrent());
        SmartDashboard.putNumber(this.motorKey()+"/DutyOut", this.getDutyOut());
    }
}
