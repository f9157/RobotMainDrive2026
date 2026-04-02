package frc.robot.subsystems;


import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


        this.main = new TalonFXSMotor(FlywheelConstants.kLeftMainFlywheelCanId);


        SlotConfigs slot = new SlotConfigs();
        
        

        MotionMagicConfigs mmConfig = Constants.FlywheelConstants.FlywheelPID.applyTalon(slot);

        var config = this.main.getInner().getConfigurator();
        
        config.apply(slot);
        config.apply(mmConfig);

        this.follower = new TalonFXSMotor(FlywheelConstants.kRightFollowFlywheelCanId);

        this.follower.follow(FlywheelConstants.kLeftMainFlywheelCanId, true);

        config.apply(slot);
        config.apply(mmConfig);
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