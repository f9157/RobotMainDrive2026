package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AutoAimCommand extends Command {

    private final DriveSubsystem  m_drive;
    private final TurretSubsystem m_turret;

    public AutoAimCommand(DriveSubsystem drive, TurretSubsystem turret) {
        m_drive  = drive;
        m_turret = turret;
        addRequirements(m_turret);
    }

    @Override
    public void initialize() {
        m_turret.setAutoAim(true);
    }

    @Override
    public void execute() {
        m_turret.updateAimAngle(m_drive.getPose2d(), FieldConstants.getHubPosition());
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setAutoAim(false);
        m_turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
