package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Targeting;
import frc.robot.subsystems.TurretSubsystem;

public class AutoTurret extends Command {
    TurretSubsystem turret;
    Targeting targeting;
    boolean feeding;
    
    public AutoTurret(TurretSubsystem turret, Targeting targeting, boolean feeding) {
        this.turret = turret;
        this.targeting = targeting;
        this.feeding = feeding;
    }

    public void execute() {
        Pose2d target = feeding? this.targeting.getNearestCorner(): this.targeting.getHubPose();
        double angle = this.targeting.getRelativeTurretAngleToTargetPose(target);
        this.turret.setAngleDegrees(angle);
    }

    public boolean isFinished() {
        return false;
    }
}
