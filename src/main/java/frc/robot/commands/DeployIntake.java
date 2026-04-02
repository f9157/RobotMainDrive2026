package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DeployIntake extends Command {
    
    IntakeSubsystem intake;

    public DeployIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public void initialize() {
        this.intake.deployIntake();
    }

    public boolean isFinished() {
        return this.intake.isDeployed(); 
    }
}
