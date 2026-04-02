package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveTeleop;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonOdometry;

public class RobotContainer {
  public static DriveSubsystem m_drive = new DriveSubsystem();

  public static final FlywheelSubsystem m_flywheel = new FlywheelSubsystem();

  public static final IntakeSubsystem m_intake = new IntakeSubsystem();

  public static final IndexerSubsystem m_indexer = new IndexerSubsystem();

  private final PhotonOdometry m_leftVision = new PhotonOdometry("left_camera", Constants.VisionConstants.kLeftCameraOffset, this.m_drive);
  private final PhotonOdometry m_rightVision = new PhotonOdometry("right_camera", Constants.VisionConstants.kRightCameraOffset, this.m_drive);


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