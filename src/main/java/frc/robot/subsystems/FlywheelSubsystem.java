package frc.robot.subsystems;


import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class FlywheelSubsystem extends SubsystemBase {

    // SparkMax main;
    TalonFXS main;
    // SparkMax follower;
    TalonFXS follower;

    StatusSignal<AngularVelocity> velocity;

    public FlywheelSubsystem() {


        this.main = new TalonFXS(FlywheelConstants.kLeftMainFlywheelCanId);


        SlotConfigs slot = new SlotConfigs();

        MotionMagicConfigs mmConfig = Constants.FlywheelConstants.FlywheelPID.applyTalon(slot);

        var config = this.main.getConfigurator();
        
        config.apply(slot);
        config.apply(mmConfig);

        this.velocity = this.main.getVelocity();

        this.follower = new TalonFXS(FlywheelConstants.kRightFollowFlywheelCanId);

        this.follower.setControl(new Follower(FlywheelConstants.kLeftMainFlywheelCanId, MotorAlignmentValue.Opposed));

        config.apply(slot);
        config.apply(mmConfig);
    }

    public void setFlywheel(double rotationsPerSecond) {
        this.main.setControl(new MotionMagicVelocityVoltage(rotationsPerSecond));
    }


    public void stopFlywheel() {
        this.main.stopMotor();
    }

    public double getFlywheelRotationsPerSecond() {
        this.velocity.refresh();
        return this.velocity.getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel/RPS", this.getFlywheelRotationsPerSecond());
    }

}