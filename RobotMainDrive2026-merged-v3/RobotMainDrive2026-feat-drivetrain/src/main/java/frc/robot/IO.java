package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ShooterConstants.HoodPosition;
import frc.robot.commands.ShootCommand;

public class IO {

    public static final XboxController m_driverController   = new XboxController(Constants.OIConstants.kDriverControllerPort);
    public static final XboxController m_operatorController = new XboxController(1);

    public static void initialize() {
        configureDriver();
        configureOperator();
    }

    private static void configureDriver() {
        new JoystickButton(m_driverController, XboxController.Button.kY.value)
            .onTrue(new InstantCommand(RobotContainer.m_drive::zeroHeading, RobotContainer.m_drive));

        new JoystickButton(m_driverController, XboxController.Button.kX.value)
            .onTrue(new InstantCommand(RobotContainer.m_drive::setZero, RobotContainer.m_drive));
    }

    private static void configureOperator() {
        RobotContainer.m_turret.setDefaultCommand(new RunCommand(() -> {
            double nudge = MathUtil.applyDeadband(m_operatorController.getRightX(), 0.05);
            if (Math.abs(nudge) > 0.0) {
                RobotContainer.m_turret.setManualAngle(
                    RobotContainer.m_turret.getTurretAngleDeg() + nudge * 2.0);
            }
            double trim = MathUtil.applyDeadband(-m_operatorController.getLeftY(), 0.1) * 10.0;
            RobotContainer.m_shooter.setRPMTrim(trim);
        }, RobotContainer.m_turret));

        new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
            .whileTrue(new ShootCommand(RobotContainer.m_drive, RobotContainer.m_turret, RobotContainer.m_shooter));

        new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
            .whileTrue(new InstantCommand(() -> {
            }));

        new JoystickButton(m_operatorController, XboxController.Button.kA.value)
            .onTrue(new InstantCommand(() -> RobotContainer.m_turret.setManualAngle(0.0), RobotContainer.m_turret));

        new JoystickButton(m_operatorController, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(() -> RobotContainer.m_shooter.setHoodPosition(HoodPosition.LOW), RobotContainer.m_shooter));

        new JoystickButton(m_operatorController, XboxController.Button.kY.value)
            .onTrue(new InstantCommand(() -> RobotContainer.m_shooter.setHoodPosition(HoodPosition.HIGH), RobotContainer.m_shooter));
    }
}
