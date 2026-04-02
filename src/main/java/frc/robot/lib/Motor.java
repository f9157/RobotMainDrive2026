package frc.robot.lib;

public interface Motor {

    public String getMotorType();

    public String getMotorName();

    public int getDeviceId();

    public void setDutyOut(double ref);

    public double getDutyOut();

    public void stop();
}
