package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants;

public class PhotonOdometry extends SubsystemBase {

    private DriveSubsystem drive;
    private String name;
    private Transform3d offset;
    private PhotonPoseEstimator photonEst;
    private PhotonCamera cam;

    PhotonOdometry(String name, Transform3d offset, DriveSubsystem drive) {
        this.name = name;
        this.offset = offset;
        this.drive = drive;
        this.photonEst = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
                offset);
        this.cam = new PhotonCamera(this.name);
    }

    private FieldObject2d getVisionDiag() {
        return this.drive.field.getObject("vision_" + this.name);
    }

    private double stdDevExp(double distance) {
        
        if (distance < VisionConstants.maxFlatDistanceMeters) {
            return Constants.VisionConstants.FlatStdDevXY;
        }

        double shiftedDistance = distance - VisionConstants.maxFlatDistanceMeters;

        return Math.exp(VisionConstants.expMultiplier * shiftedDistance) - 1 + VisionConstants.FlatStdDevXY;
    }

    private Matrix<N3, N1> computeStdDev(EstimatedRobotPose pose, PhotonPipelineResult res) {
        double distance = res.getBestTarget().bestCameraToTarget.getTranslation().getDistance(Translation3d.kZero);
        
        double stdDevXY = this.stdDevExp(distance);

        double stdDevTheta = 1;

        double[] array = {stdDevXY, stdDevXY, stdDevTheta};

        return new Matrix<N3,N1>(new SimpleMatrix(array));
    }
    @Override
    public void periodic() {

        List<PhotonPipelineResult> results = this.cam.getAllUnreadResults();

        for (PhotonPipelineResult res : results) {
            Optional<EstimatedRobotPose> maybePoseEst = this.photonEst.estimateAverageBestTargetsPose(res);

            if (maybePoseEst.isEmpty()) {
                this.getVisionDiag().setPoses();
                continue;
            }

            EstimatedRobotPose poseEst = maybePoseEst.get();

            this.getVisionDiag().setPose(poseEst.estimatedPose.toPose2d());

            this.drive.odometry.addVisionMeasurement(poseEst.estimatedPose.toPose2d(), poseEst.timestampSeconds,
                    this.computeStdDev(poseEst, res));

        }

    }

}
