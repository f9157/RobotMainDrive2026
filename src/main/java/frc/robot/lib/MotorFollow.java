package frc.robot.lib;

public interface MotorFollow extends Motor {
    public void follow(int otherCanId, boolean invert);
}
