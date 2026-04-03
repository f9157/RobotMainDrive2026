// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Targeting;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import choreo.auto.*;

public final class Autos {

  private final AutoFactory factory;

  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public Autos(DriveSubsystem drive) {
    this.factory = new AutoFactory(drive::getPose2d, drive::resetOdometry, drive::followTrajectory, false, drive);
  }

  private void first(AutoRoutine routine, AutoTrajectory first) {
    routine.active().onTrue(
        Commands.sequence(
            first.resetOdometry(),
            first.cmd()));
  }

  private AutoTrajectory FullFeedPath(AutoRoutine routine) {
    return routine.trajectory("TotalFeed");
  }

  private AutoTrajectory HalfFeedPath(AutoRoutine routine) {
    return routine.trajectory("HalfFeed");
  }

  private AutoTrajectory shootAlianceStorage(AutoRoutine routine) {
    return routine.trajectory("AllianceStorage");
  }

  public AutoRoutine leftFeed(boolean full, TurretSubsystem turret, FlywheelSubsystem flywheel,
      IndexerSubsystem indexer, Targeting targeting, IntakeSubsystem intake) {
    AutoRoutine routine = this.factory.newRoutine("LeftFullFeed");

    AutoTrajectory rabidPath = full ? this.FullFeedPath(routine) : this.FullFeedPath(routine);

    this.first(routine, rabidPath);

    routine.active().onTrue(new AutoAim(turret, flywheel, indexer, targeting, true)
        .alongWith(new DeployIntake(intake).andThen(new RunIntake(intake))));

    return routine;
  }

  public AutoRoutine shootThenLeftFeed(boolean full, TurretSubsystem turret, FlywheelSubsystem flywheel,
      IndexerSubsystem indexer, Targeting targeting, IntakeSubsystem intake) {
    AutoRoutine routine = this.factory.newRoutine("ShootThenLeftFeed");

    AutoTrajectory shootStorage = this.shootAlianceStorage(routine);

    AutoTrajectory rabidPath = full ? this.FullFeedPath(routine) : this.FullFeedPath(routine);

    this.first(routine, shootStorage);

        routine.active().onTrue(new AutoAim(turret, flywheel, indexer, targeting, false)
        .alongWith(new DeployIntake(intake).andThen(new RunIntake(intake))));

    routine.anyDone(shootStorage).onTrue(new AutoAim(turret, flywheel, indexer, targeting, true));

    shootStorage.chain(rabidPath);

    return routine;
  }
}
