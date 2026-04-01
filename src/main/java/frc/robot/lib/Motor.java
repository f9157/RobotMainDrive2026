package frc.robot.lib;

public interface Motor {

    public String getMotorType();

    public int getDeviceId();

    public void setDutyOut(double ref);

    public double getDutyOut();
}
