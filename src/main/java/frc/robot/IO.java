package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StopIntake;

public class IO {

    public static final CommandXboxController m_driverController = new CommandXboxController(Constants.IOConstants.kDriverControllerPort);

    public static final CommandXboxController m_operatorController = new CommandXboxController(
            Constants.IOConstants.kOperatorCOntrollerPort);

    public static void initialize() {

        m_driverController.y()
                .onTrue(new InstantCommand(RobotContainer.m_drive::zeroHeading, RobotContainer.m_drive));

        m_driverController.x()
                .onTrue(new InstantCommand(RobotContainer.m_drive::setZero, RobotContainer.m_drive));


        m_driverController.a()
                .onTrue(new DeployIntake(RobotContainer.m_intake));

        m_driverController.b()
                .onTrue(new RetractIntake(RobotContainer.m_intake));

        m_driverController.leftTrigger()
                .onTrue(new RunIntake(RobotContainer.m_intake)).onFalse(new StopIntake(RobotContainer.m_intake));
        

    }
}
