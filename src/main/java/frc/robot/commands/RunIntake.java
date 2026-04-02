package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {
    
    IntakeSubsystem intake;

    public RunIntake(IntakeSubsystem intake) {
        this.intake = intake;
    }


    public void initialize() {
        this.intake.startIntaking();
    }

    public boolean isFinished() {
        return true;
    }
}
