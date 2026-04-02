package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeting;
import frc.robot.subsystems.FlywheelSubsystem;

public class AutoFlywheel extends Command {
    
    FlywheelSubsystem flywheel;
    Targeting targeting;
    boolean feeding;

    public AutoFlywheel(FlywheelSubsystem flywheel, Targeting targeting, boolean feeding) {
        this.flywheel = flywheel;
        this.targeting = targeting;
        this.feeding = feeding;
    }

    public void execute() {
        double speed = feeding? this.targeting.getFlywheelSpeedInterRPSToCorner(): this.targeting.getFlywheelSpeedInterRPSToHub();

        this.flywheel.setFlywheel(speed);
    }

    public boolean isFinished() {
        return false;
    }
}
