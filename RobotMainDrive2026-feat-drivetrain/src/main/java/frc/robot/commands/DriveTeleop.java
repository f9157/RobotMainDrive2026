package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTeleop extends Command {

    DriveSubsystem drive;

    public DriveTeleop(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double xInput   = -IO.m_driverController.getLeftX();
        double yInput   = -IO.m_driverController.getLeftY();
        double rotInput = -IO.m_driverController.getRightX();

        xInput   = drive.applyDeadband(xInput);
        yInput   = drive.applyDeadband(yInput);
        rotInput = drive.applyDeadband(rotInput);

        double xSpeed = yInput   * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeed = xInput   * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
        double rot    = rotInput * Constants.DriveConstants.kMaxAngularSpeedRadPerSec;

        drive.drive(xSpeed, ySpeed, rot, true);

        if (RobotContainer.m_vision.hasTarget() && RobotContainer.m_vision.getLatestPose() != null) {
            drive.addVisionMeasurement(
                RobotContainer.m_vision.getLatestPose(),
                RobotContainer.m_vision.getLatestTimestamp());
        }
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted) {}
}
