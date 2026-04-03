package frc.robot.lib;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public interface SimMotor extends Motor  {

    public double getSimOutputVoltage();

    public void feedSimulatedSensorData(double positionRotations, double velocityRotations);
}
