package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lib.SparkMotor;
import frc.robot.lib.TalonFXSMotor;

public class IntakeSubsystem extends SubsystemBase {
    
    TalonFXSMotor deployMotor;

    SparkMotor wheelMain;
    SparkMotor wheelFollow;


    public IntakeSubsystem() {


        this.deployMotor = new TalonFXSMotor(IntakeConstants.kDeployMotorCanId, "IntakeDeploy");

        this.wheelMain = new SparkMotor(IntakeConstants.kWheelMainMotorCanId, "IntakeWheelMain", false);

        this.wheelFollow = new SparkMotor(IntakeConstants.kWheelFollowMotorCanId, "IntakeWheelFollow", false);

        Constants.IntakeConstants.WheelPID.applySparkMax(this.wheelMain);
    

        this.wheelFollow.follow(IntakeConstants.kWheelMainMotorCanId, false);
        
        

    }



    public void retractIntake() {
        this.deployMotor.setPosition(IntakeConstants.kIntakeRetractPosition);
    }

    public void deployIntake() {
        this.deployMotor.setPosition(IntakeConstants.kIntakeDeployPosition);
    }

    public boolean isDeployed() {
        return Math.abs(this.deployMotor.getPosition() - IntakeConstants.kIntakeDeployPosition) < IntakeConstants.kIntakeDeployTolerance;
    }

    public boolean isRetracted() {
        return Math.abs(this.deployMotor.getPosition() - IntakeConstants.kIntakeRetractPosition) < IntakeConstants.kIntakeRetractTolerance;
    }

    public void startIntaking() {
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
