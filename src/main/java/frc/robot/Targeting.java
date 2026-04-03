package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.TargetingConstants;
import frc.robot.Constants.PositionConstants;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.DriveSubsystem;

public class Targeting {

    public static class AutoAimingState {
        public static boolean turretAimed = false;
        public static boolean hoodAimed = false;
        public static boolean flywheelWithinTolerance = false;
    }

    public AutoAimingState autoAimingState = new AutoAimingState();

    private final DriveSubsystem drivetrain;
    private Pose2d hubPose;
    private final Field2d field = new Field2d();
    private int shotDistanceFudgeSteps = 0;

    public Targeting(DriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        TargetingConstants.flyWheelSpeedLookupInitializer();
        SmartDashboard.putData("Aim/Field2d", field);

        if (AllianceZoneUtil.isBlueAlliance()) {
            hubPose = PositionConstants.BlueHubPose;
        } else {
            hubPose = PositionConstants.RedHubPose;
        }
        SmartDashboard.putNumber("Aim/ShotDistanceFudge", shotDistanceFudgeSteps);
    }

    private void updateHubPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            hubPose = PositionConstants.BlueHubPose;
        } else {
            hubPose = PositionConstants.RedHubPose;
        }
    }

    public Pose2d getHubPose() {
        return transformPose(hubPose, PositionConstants.DeltaZ, getBallExitVelocityMPS(), getTargetAngleMinVelocity());
    }

    public Pose2d getCornerPose() {
        Pose2d corner = getNearestCorner();
        return transformPose(corner, -PositionConstants.RobotZ, getBallExitVelocityMPSToCorner(), getTargetAngleToCorner());
    }

    private Pose2d transformPose(Pose2d pos, double z, double vb, double theta) {
        Translation2d vr = getFieldRelativeVelocity();
        double vx = vr.getX();
        double vy = vr.getY();

        double launchAngleRadians = Math.toRadians(90.0 - theta);
        double t = getFlightTime(
                launchAngleRadians,
                vb,
                z);

        if (!Double.isFinite(t) || t < 0.0) {
            return pos;
        }

        double dx = vx * t;
        double dy = vy * t;

        return pos.transformBy(new Transform2d(-dx, -dy, Rotation2d.kZero));
    }

    private Translation2d getFieldRelativeVelocity() {
        ChassisSpeeds robotRelative = drivetrain.getRobotRelativeSpeeds();
        Rotation2d heading = drivetrain.getHeading();

        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelative.vxMetersPerSecond,
                robotRelative.vyMetersPerSecond,
                robotRelative.omegaRadiansPerSecond,
                heading);

        return new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond);
    }

    public static double getFlightTime(double angleRadians, double initial_velocity, double height_diff) {
        double g = TargetingConstants.gravity;

        double verticalVelocity = initial_velocity * Math.sin(angleRadians);

        double discriminant = Math.pow(verticalVelocity, 2) - 2.0 * g * height_diff;

        if (discriminant < 0) {
            return Double.NaN;
        }

        return (verticalVelocity + Math.sqrt(discriminant)) / g;
    }

    public double getRobotDistance(Pose2d Pose1, Pose2d Pose2) {
        return Pose2.getTranslation().getDistance(Pose1.getTranslation());
    }

    public double getRobotToHub() {
        double rawDistanceMeters = getRobotDistance(hubPose, drivetrain.getPose2d());
        double fudgeDistanceMeters = shotDistanceFudgeSteps * TargetingConstants.FudgeConstants.shotDistanceFudgeStepMeters;
        double fudgedDistanceMeters = rawDistanceMeters + fudgeDistanceMeters;

        // Prevent distance from every going to 0 or negative to prevent divide by zero errors in calculations
        return Math.max(0.001, fudgedDistanceMeters);
    }

    public double getRobotToNearestCorner() {
        Pose2d corner = getNearestCorner();
        return getRobotDistance(corner, drivetrain.getPose2d());
    }

    public double getRobotToVirtualHub() {
        return getRobotDistance(getHubPose(), drivetrain.getPose2d());
    }

    public double getRobotToVirtualCorner() {
        return getRobotDistance(getCornerPose(), drivetrain.getPose2d());
    }

    public void adjustShotDistanceFudgeSteps(int deltaSteps) {
        shotDistanceFudgeSteps += deltaSteps;
        SmartDashboard.putNumber("Aim/ShotDistanceFudge", shotDistanceFudgeSteps);
    }

    public Pose2d getNearestCorner() {
        if (AllianceZoneUtil.isBlueAlliance()) {
            if (getRobotInTopHalfOfField()) {
                return AutoConstants.blueAllianceTop; // blue alliance is the left corner of the blue alliance
            } else {
                return AutoConstants.blueAllianceBottom; // blue alliance bottom is the right corner of the blue
                                                         // alliance
            }
        } else {
            if (getRobotInTopHalfOfField()) {
                return AutoConstants.redAllianceTop; // red alliance top is the left corner of the red alliance
            } else {
                return AutoConstants.redAllianceBottom; // red alliance bottom is the right corner of the red alliance
            }
        }
    }

    public double getTargetAngleMinVelocity() {
        double x = getRobotToHub();
        double z = PositionConstants.DeltaZ;
        return getAngle(x, z) - 11;
    }

    public double getTargetAngleToCorner() {
        double x = getRobotToNearestCorner();
        double z = -PositionConstants.RobotZ; // corners are at the same height as the robot, so we want to shoot slightly upwards to account for that
        return getAngle(x, z);
    }

    private double getAngle(double x, double z) {
        double targetDegrees = 0.0;

        double theta = Math.atan((z + Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2))) / x);

        targetDegrees = Math.toDegrees(theta);
        targetDegrees = 90 - targetDegrees;

        return targetDegrees;
    }

    private double getBallVelocity(double distance, double height) {
        double trajDistance = Math.sqrt(Math.pow(distance, 2) + Math.pow(height, 2));
        double vel_sq = TargetingConstants.gravity * (trajDistance + height);
        return Math.sqrt(vel_sq);
    }

    public double getBallExitVelocityMPS() {
        double distance = this.getRobotToHub();
        double height = PositionConstants.DeltaZ;
        return getBallVelocity(distance, height);
    }

    // Oh boy do I love these naming conventions

    public double getBallExitVelocityMPSToCorner() {
        double distance = this.getRobotToNearestCorner();
        return getBallVelocity(distance, 0);
    }

    public double getFlywheelSpeedInterRPSToHub() {
        // linear
        double distance = this.getRobotToVirtualHub();
        return TargetingConstants.flywheelSpeedLookup.get(distance);
    }

    public double getFlywheelSpeedInterRPSToCorner() {
        // linear
        double distance = this.getRobotToVirtualCorner();
        return TargetingConstants.flywheelSpeedLookup.get(distance);
    }

    public boolean inAllianceZone() {
        return AllianceZoneUtil.inAllianceZone(drivetrain.getPose2d().getX());
    }

    public boolean notInAllianceZone() {
        return !inAllianceZone();
    }

    public double getRelativeTurretAngleToTargetPose(Pose2d targetPose) {
        Pose2d robotPose = drivetrain.getPose2d();
        Translation2d robotPoint = robotPose.getTranslation();
        Translation2d targetPoint = targetPose.getTranslation();

        // 1. Calculate the absolute field-relative angle to the hub
        // This is the angle from the robot's (x,y) to the hub's (x,y)
        Rotation2d fieldRelativeAngle = targetPoint.minus(robotPoint).getAngle();

        // 2. Subtract the robot's heading to make it robot-relative
        // If robot is at 20 deg and target is at 30 deg, turret needs to be at 10 deg.
        // Field angle is 0
        // Robot angle is 90 CCW
        // Turret angle is -90 CCW (0 is straight on)

        Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.getRotation());

        return MathUtil.inputModulus(robotRelativeAngle.getDegrees(), -180, 180);
    }

    public double getTurretToleranceDegrees() {

        double distance = this.getRobotToHub();

        double m = Math.abs(distance / (TargetingConstants.goalRadius - TargetingConstants.fuelRadius));

        double c = Math.atan(m);

        return Math.toDegrees((Math.PI / 2) - c);
    }

    public boolean getRobotInTopHalfOfField() {
        return drivetrain.getPose2d().getY() > (4.03);
    }

    // Dashboard data
    public void sendData() {
        this.updateHubPose();
        SmartDashboard.putNumber("Aim/OptimalAngle", getTargetAngleMinVelocity());
        SmartDashboard.putNumber("Aim/robotDistance", getRobotToHub());
        SmartDashboard.putNumber("Aim/TurretAngleToHub", getRelativeTurretAngleToTargetPose(getHubPose()));
        SmartDashboard.putNumber("Aim/Optimal MPS", getBallExitVelocityMPS());
        SmartDashboard.putBoolean("Aim/InAllianceZone", inAllianceZone());

        Transform2d diff = getHubPose().minus(hubPose);
        SmartDashboard.putNumber("Aim/HubPoseDiffX", diff.getTranslation().getX());
        SmartDashboard.putNumber("Aim/HubPoseDiffY", diff.getTranslation().getY());
        SmartDashboard.putNumber("Aim/HubPoseDiffRotation", diff.getRotation().getDegrees());
        field.getObject("aimPoint").setPose(getHubPose());
    }
}
