package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveTeleop;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    public static final DriveSubsystem   m_drive   = new DriveSubsystem();
    public static final TurretSubsystem  m_turret  = new TurretSubsystem();
    public static final ShooterSubsystem m_shooter = new ShooterSubsystem();
    public static final VisionSubsystem  m_vision  = new VisionSubsystem();

    public RobotContainer() {
        m_drive.setDefaultCommand(new DriveTeleop(m_drive));
        IO.initialize();
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
