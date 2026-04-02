package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class IO {

    public static final XboxController m_driverController = new XboxController(Constants.IOConstants.kDriverControllerPort);

    public static final XboxController m_operatorController = new XboxController(
            Constants.IOConstants.kOperatorCOntrollerPort);

    public static void initialize() {

        new JoystickButton(m_driverController, XboxController.Button.kY.value)
                .onTrue(new InstantCommand(RobotContainer.m_drive::zeroHeading, RobotContainer.m_drive));

        new JoystickButton(m_driverController, XboxController.Button.kX.value)
                .onTrue(new InstantCommand(RobotContainer.m_drive::setZero, RobotContainer.m_drive));

    }
}
