package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Targeting;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAim extends ParallelCommandGroup {
    
    public AutoAim(TurretSubsystem turret, FlywheelSubsystem flywheel, IndexerSubsystem indexer, Targeting targeting, boolean feeding) {
        addCommands(
            new AutoFlywheel(flywheel, targeting, feeding),
            new AutoTurret(turret, targeting, feeding),
            new AutoIndex(indexer, feeding)
        );
    }
}
