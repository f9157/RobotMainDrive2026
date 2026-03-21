package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FlywheelSubsystem extends SubsystemBase {

    SparkMax main;
    SparkMax follower;

    SparkClosedLoopController controller;

    RelativeEncoder encoder;

    public FlywheelSubsystem() {

        ClosedLoopConfig flywheelPID = new ClosedLoopConfig().pid(0.0001, 0, 0)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig.apply(flywheelPID);
        flywheelConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);

        this.main = new SparkMax(Constants.FlywheelConstants.kLeftMainFlywheelCanId, MotorType.kBrushless);

        this.main.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.follower = new SparkMax(Constants.FlywheelConstants.kRightFollowFlywheelCanId, MotorType.kBrushless);


        flywheelConfig.follow(Constants.FlywheelConstants.kLeftMainFlywheelCanId, true);
        this.follower.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.controller = this.main.getClosedLoopController();
        this.encoder = this.main.getEncoder();
    }

    public void setFlywheel(double rpm) {
        this.controller.setSetpoint(rpm, ControlType.kVelocity);
    }


    public void stopFlywheel() {
        this.main.stopMotor();
    }

    public double getFlywheelRPM() {
        return this.encoder.getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel/RPM", this.getFlywheelRPM());
    }

}