package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimTurret;
import frc.robot.commands.AutoAim;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StopIntake;

public class IO {

        public static final CommandXboxController m_driverController = new CommandXboxController(
                        Constants.IOConstants.kDriverControllerPort);

        public static final CommandXboxController m_operatorController = new CommandXboxController(
                        Constants.IOConstants.kOperatorCOntrollerPort);

        public static void initialize() {

                m_driverController.y()
                                .onTrue(new InstantCommand(RobotContainer.m_drive::zeroHeading,
                                                RobotContainer.m_drive));

                m_driverController.x()
                                .onTrue(new InstantCommand(RobotContainer.m_drive::setZero, RobotContainer.m_drive));

                m_driverController.a()
                                .onTrue(new DeployIntake(RobotContainer.m_intake));

                m_driverController.b()
                                .onTrue(new RetractIntake(RobotContainer.m_intake));

                m_driverController.leftTrigger()
                                .onTrue(new RunIntake(RobotContainer.m_intake))
                                .onFalse(new StopIntake(RobotContainer.m_intake));

                m_driverController.povUp()
                                .onTrue(new AimTurret(RobotContainer.m_turret, 0));

                m_driverController.povLeft()
                                .onTrue(new AimTurret(RobotContainer.m_turret, 90));

                m_driverController.povRight()
                                .onTrue(new AimTurret(RobotContainer.m_turret, -90));

                m_driverController.povDown()
                                .onTrue(new InstantCommand(RobotContainer.m_indexer::runIndexer, RobotContainer.m_indexer))
                                .onFalse(new InstantCommand(RobotContainer.m_indexer::stopIndexer, RobotContainer.m_indexer));


                m_driverController.leftBumper()
                                .onTrue(new AutoAim(RobotContainer.m_turret, RobotContainer.m_flywheel, RobotContainer.m_indexer,
                                                RobotContainer.m_targeting, false));

                m_driverController.rightBumper()
                                .onTrue(new AutoAim(RobotContainer.m_turret, RobotContainer.m_flywheel, RobotContainer.m_indexer,
                                                RobotContainer.m_targeting, true));

        }
}
