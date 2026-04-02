package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.TalonSRXMotor;

public class HoodSubsystem extends SubsystemBase {
    
    PIDController actuatorPID = new PIDController(0,0,0);

    TalonSRXMotor actuator;

    public HoodSubsystem() {

        this.actuator = new TalonSRXMotor(Constants.HoodConstants.kActuatorCanId, "Actuator", false);

    }

    
    @Override
    public void periodic() {

    }
}
