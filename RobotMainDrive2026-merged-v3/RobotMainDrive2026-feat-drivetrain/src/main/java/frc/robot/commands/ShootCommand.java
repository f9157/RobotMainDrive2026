package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShootCommand extends Command {

    private final DriveSubsystem   m_drive;
    private final TurretSubsystem  m_turret;
    private final ShooterSubsystem m_shooter;

    public ShootCommand(DriveSubsystem drive, TurretSubsystem turret, ShooterSubsystem shooter) {
        m_drive   = drive;
        m_turret  = turret;
        m_shooter = shooter;
        addRequirements(m_turret, m_shooter);
    }

    @Override
    public void initialize() {
        m_turret.setAutoAim(true);
        m_shooter.setShooterActive(true);
    }

    @Override
    public void execute() {
        m_turret.updateAimAngle(m_drive.getPose2d(), FieldConstants.getHubPosition());
        double distance = m_turret.getDistanceToTarget(m_drive.getPose2d(), FieldConstants.getHubPosition());
        m_shooter.setDistanceTarget(distance);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setAutoAim(false);
        m_turret.stop();
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
