package frc.robot.subsystems;

import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonCamera extends SubsystemBase {

    private DriveSubsystem drive;
    private String name;
    private Transform3d offset;
    private PhotonPoseEstimator photonEst;

    PhotonCamera(String name, Transform3d offset, DriveSubsystem drive) {
        this.name = name;
        this.offset = offset;
        this.drive = drive;
        this.photonEst = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
                offset);
    }

    @Override
    public void periodic() {
        this.photonEst.estimateAverageBestTargetsPose(null)
    }

}
