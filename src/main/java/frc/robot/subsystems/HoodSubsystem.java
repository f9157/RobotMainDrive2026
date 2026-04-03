package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.TalonSRXMotor;

public class HoodSubsystem extends SubsystemBase {
    
    PIDController actuatorPID = new PIDController(0,0,0);

    TalonSRXMotor actuator;

    public HoodSubsystem() {

        this.actuator = new TalonSRXMotor(Constants.HoodConstants.kActuatorCanId, "Actuator", false);

        this.actuator.getInner().configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 30);

        this.actuator.getInner().config_kP(0, 1.0);
        this.actuator.getInner().config_kI(0, 0);
        this.actuator.getInner().config_kD(0, 0);

    }

    private void setActuatorPos(double ref) {
        double sensorUnits = ref * 1023;
        this.actuator.setPosition(sensorUnits);
    }

    public void setAngleDegrees(double degrees) {
        // clamp here between min and max
        // find 0-1 from Delta degrees
        // pass into setActuatorPosition
    }

    
    @Override
    public void periodic() {
        double sensorUnits = this.actuator.getInner().getSelectedSensorPosition();
        SmartDashboard.putNumber("Hood/Actuator_position", sensorUnits);
    }
}
