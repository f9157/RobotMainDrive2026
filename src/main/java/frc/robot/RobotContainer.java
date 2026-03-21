package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveTeleop;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  public static DriveSubsystem m_drive = new DriveSubsystem();

  public RobotContainer() {
    configureDefaultCommands();
    IO.initialize();
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(new DriveTeleop(m_drive));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}