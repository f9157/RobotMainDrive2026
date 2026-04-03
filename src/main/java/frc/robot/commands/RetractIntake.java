package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntake extends Command {
    
    IntakeSubsystem intake;
    public RetractIntake(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    public void initialize() {
        this.intake.retractIntake();
    }

    public boolean isFinished() {
        return this.intake.isRetracted();
    }
}

