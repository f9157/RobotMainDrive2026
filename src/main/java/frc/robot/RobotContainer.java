package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureDefaultCommands();
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(
        new RunCommand(() -> {
          double xInput = -m_driverController.getLeftX();
          double yInput = -m_driverController.getLeftY();
          double rotInput = -m_driverController.getRightX();

          xInput = m_drive.applyDeadband(xInput);
          yInput = m_drive.applyDeadband(yInput);
          rotInput = m_drive.applyDeadband(rotInput);
          new JoystickButton(m_driverController, XboxController.Button.kY.value)
              .onTrue(new InstantCommand(m_drive::zeroHeading, m_drive));

          new JoystickButton(m_driverController, XboxController.Button.kX.value)
              .onTrue(new InstantCommand(m_drive::setZero, m_drive));
          // Field is X axis down the field, Y axis along the alliance station
          double xSpeed = yInput * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
          double ySpeed = xInput * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
          double rot = rotInput * Constants.DriveConstants.kMaxAngularSpeedRadPerSec;

          m_drive.drive(xSpeed, ySpeed, rot, true);
        }, m_drive));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}