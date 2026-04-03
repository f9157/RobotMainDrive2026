package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeting;
import frc.robot.subsystems.IndexerSubsystem;

public class AutoIndex extends Command {

    IndexerSubsystem indexer;
    boolean feeding;
    
    public AutoIndex(IndexerSubsystem indexer, boolean feeding) {
        this.indexer = indexer;
        this.feeding = feeding;
    }

    public boolean canShoot() {
        boolean canShoot = false;
        if (feeding) {
            canShoot = true;
        } else {
            canShoot = Targeting.AutoAimingState.flywheelWithinTolerance && Targeting.AutoAimingState.turretAimed && Targeting.AutoAimingState.hoodAimed;
        }
        return canShoot;
    }

    public void execute() {
        boolean canShoot = this.canShoot();

        if (canShoot) {
            this.indexer.runIndexer();
        } else {
            this.indexer.stopIndexer();
        }
    }

    public boolean isFinished() {
        return false;
    }
}
