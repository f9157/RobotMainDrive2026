package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTeleop extends Command {

    private DriveSubsystem drive;

    public DriveTeleop(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    public void execute() {

        double xInput = -IO.m_driverController.getLeftX();
          double yInput = -IO.m_driverController.getLeftY();
          double rotInput = -IO.m_driverController.getRightX();

          xInput = this.drive.applyDeadband(xInput);
          yInput = this.drive.applyDeadband(yInput);
          rotInput = this.drive.applyDeadband(rotInput);

        // Field is X axis down the field, Y axis along the alliance station
        double xSpeed = yInput * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeed = xInput * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        double rot = rotInput * Constants.DriveConstants.kMaxAngularSpeedRadPerSec;

        this.drive.drive(xSpeed, ySpeed, rot, true);
    }

    public boolean isFinished() {
        return false;
    }

}
