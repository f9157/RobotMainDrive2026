package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.lib.TalonSRXMotor;

public class IndexerSubsystem extends SubsystemBase {
    

    TalonSRXMotor motor;

    public IndexerSubsystem() {

        this.motor = new TalonSRXMotor(Constants.IndexerConstants.kMotorCanId, false);
    }

    public void runIndexer() {
        this.motor.setDutyOut(IndexerConstants.kIndexerSpeed);
    }

    public void stopIndexer() {
        this.motor.stop();
    }
}
