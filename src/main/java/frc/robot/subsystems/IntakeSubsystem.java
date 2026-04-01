package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    TalonFXS deployMotor;

    SparkMax wheelMain;
    SparkMax wheelFollow;


    public IntakeSubsystem() {


        this.deployMotor = new TalonFXS(IntakeConstants.kDeployMotorCanId);

        this.wheelMain = new SparkMax(IntakeConstants.kWheelMainMotorCanId, MotorType.kBrushless);

        this.wheelFollow = new SparkMax(IntakeConstants.kWheelFollowMotorCanId, MotorType.kBrushless);


    }



    public void retractIntake() {

    }

    public void deployIntake() {

    }

    public void intake() {

    }

    public void stopIntaking() {

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/deploy_position", )
    }

}
