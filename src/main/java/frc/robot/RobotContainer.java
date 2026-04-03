package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.AutoAim;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTeleop;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonOdometry;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  public static DriveSubsystem m_drive = new DriveSubsystem();

  public static final FlywheelSubsystem m_flywheel = new FlywheelSubsystem();

  public static final IntakeSubsystem m_intake = new IntakeSubsystem();

  public static final IndexerSubsystem m_indexer = new IndexerSubsystem();

  public static final TurretSubsystem m_turret = new TurretSubsystem();

  public static final Targeting m_targeting = new Targeting(m_drive);

  private static final PhotonOdometry m_leftVision = new PhotonOdometry("left_camera", Constants.VisionConstants.kLeftCameraOffset, m_drive);
  private static final PhotonOdometry m_rightVision = new PhotonOdometry("right_camera", Constants.VisionConstants.kRightCameraOffset, m_drive);

  private final Autos autos = new Autos(m_drive);

  private final AutoChooser chooser = new AutoChooser();

  public RobotContainer() {
    configureDefaultCommands();
    setupAutos();
    IO.initialize();
  }

  private void setupAutos() {
    this.chooser.addRoutine("LeftHalfFeed", this::leftHalfFeed);
    this.chooser.addRoutine("LeftFullFeed", this::leftFullFeed);
    this.chooser.addRoutine("StorageLeftHalfFeed", this::storageLeftHalfFeed);
    this.chooser.addRoutine("StorageLeftFullFeed", this::storageLeftFullFeed);
    SmartDashboard.putData(this.chooser);

    RobotModeTriggers.autonomous().onTrue(this.chooser.selectedCommandScheduler());
  }

  private AutoRoutine leftHalfFeed() {
    return this.autos.leftFeed(false, m_turret, m_flywheel, m_indexer, m_targeting, m_intake);
  }

  private AutoRoutine leftFullFeed() {
    return this.autos.leftFeed(true, m_turret, m_flywheel, m_indexer, m_targeting, m_intake);
  }

  private AutoRoutine storageLeftHalfFeed() {
    return this.autos.shootThenLeftFeed(false, m_turret, m_flywheel, m_indexer, m_targeting, m_intake);
  }

  private AutoRoutine storageLeftFullFeed() {
    return this.autos.shootThenLeftFeed(true, m_turret, m_flywheel, m_indexer, m_targeting, m_intake);
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(new DriveTeleop(m_drive));
    CommandScheduler.getInstance().schedule(new AutoAim(m_turret, m_flywheel, m_indexer, m_targeting, false));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}