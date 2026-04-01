package frc.robot.lib;

public interface PidMotor extends Motor {

    public void setPosition(double position);

    public double getPosition();

    public void setVelocity(double velocity);

    public double getVelocity();

    public double getCurrent();

}
