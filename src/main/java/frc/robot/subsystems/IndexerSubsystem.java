package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.lib.VictorSPXMotor;

public class IndexerSubsystem extends SubsystemBase {
    

    VictorSPXMotor motor;

    public IndexerSubsystem() {

        this.motor = new VictorSPXMotor(Constants.IndexerConstants.kMotorCanId, "Indexer", false);
    }

    public void runIndexer() {
        this.motor.setDutyOut(IndexerConstants.kIndexerSpeed);
    }

    public void stopIndexer() {
        this.motor.stop();
    }
}
