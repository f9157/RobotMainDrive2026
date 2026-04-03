package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntake extends Command {
    
    IntakeSubsystem intake;

    public StopIntake(IntakeSubsystem intake) {
        this.intake = intake;
    }


    public void initialize() {
        this.intake.stopIntaking();
    }

    public boolean isFinished() {
        return true;
    }
}
