package frc.robot.subsystems;


import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.lib.TalonFXSMotor;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

public class FlywheelSubsystem extends SubsystemBase {

    TalonFXSMotor main;
    TalonFXSMotor follower;

    StatusSignal<AngularVelocity> velocity;

    public FlywheelSubsystem() {


        this.main = new TalonFXSMotor(FlywheelConstants.kLeftMainFlywheelCanId, "FlywheelLeft", TalonFXSMotor.MotorType.getNeo());


        this.main.apply(FlywheelConstants.FlywheelPID);

        this.follower = new TalonFXSMotor(FlywheelConstants.kRightFollowFlywheelCanId, "FlywheelRight", TalonFXSMotor.MotorType.getNeo());

        this.follower.follow(FlywheelConstants.kLeftMainFlywheelCanId, true);

        this.follower.apply(FlywheelConstants.FlywheelPID);
    }

    public void setFlywheel(double rotationsPerSecond) {
        this.main.setVelocity(rotationsPerSecond);
    }


    public void stopFlywheel() {
        this.main.stop();
    }

    public double getFlywheelRotationsPerSecond() {
        return this.main.getVelocity();
    }

    @Override
    public void periodic() {
        this.main.postMotorDiagnostics();
    }

}