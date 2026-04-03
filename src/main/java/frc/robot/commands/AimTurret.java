package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretSubsystem;

public class AimTurret extends Command {
    TurretSubsystem turret;
    double angleDegrees;
    public AimTurret(TurretSubsystem turret, double angleDegrees) {
        this.turret = turret;
        this.angleDegrees = angleDegrees;
    }

    public void initialize() {
        this.turret.setAngleDegrees(angleDegrees);
    }

    public boolean isFinished() {
        return this.turret.atTargetAngle(TurretConstants.kTurretStaticTolerance);
    }
}
