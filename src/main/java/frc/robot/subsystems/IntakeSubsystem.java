package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lib.SparkMotor;
import frc.robot.lib.TalonFXSMotor;

public class IntakeSubsystem extends SubsystemBase {
    
    TalonFXSMotor deployMotor;

    SparkMotor wheelMain;
    SparkMotor wheelFollow;


    public IntakeSubsystem() {


        this.deployMotor = new TalonFXSMotor(IntakeConstants.kDeployMotorCanId);

        this.wheelMain = new SparkMotor(IntakeConstants.kWheelMainMotorCanId, false);

        this.wheelFollow = new SparkMotor(IntakeConstants.kWheelFollowMotorCanId, false);

        this.wheelFollow.follow(IntakeConstants.kWheelMainMotorCanId, false);
        

    }



    public void retractIntake() {
        this.deployMotor.setPosition(IntakeConstants.kIntakeRetractPosition);
    }

    public void deployIntake() {
        this.deployMotor.setPosition(IntakeConstants.kIntakeDeployPosition);
    }

    public void intake() {
        this.wheelMain.setVelocity(IntakeConstants.kIntakeVelocity);
    }

    public void stopIntaking() {
        this.wheelMain.stop();
    }

    @Override
    public void periodic() {
        this.deployMotor.postMotorDiagnostics();
        this.wheelMain.postMotorDiagnostics();
    }

}
