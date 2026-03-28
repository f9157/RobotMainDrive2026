package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera        m_camera;
    private final PhotonPoseEstimator m_poseEstimator;

    private Pose2d  m_latestEstimatedPose = null;
    private double  m_latestTimestamp     = 0;
    private boolean m_hasTarget           = false;

    public VisionSubsystem() {
        m_camera = new PhotonCamera(VisionConstants.kCameraName);
        m_poseEstimator = new PhotonPoseEstimator(
            FieldConstants.kFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            VisionConstants.kCameraToRobot);
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        var result = m_camera.getLatestResult();
        m_hasTarget = false;

        if (!result.hasTargets()) {
            publishDashboard("no targets");
            return;
        }

        List<PhotonTrackedTarget> hubTargets = result.getTargets().stream()
            .filter(t -> VisionConstants.getValidTagIds().contains(t.getFiducialId()))
            .toList();

        if (hubTargets.isEmpty()) {
            publishDashboard("wrong tags visible");
            return;
        }

        if (hubTargets.size() == 1) {
            PhotonTrackedTarget tag = hubTargets.get(0);
            double ambiguity = tag.getPoseAmbiguity();
            if (ambiguity < 0 || ambiguity > VisionConstants.kMaxAmbiguity) {
                publishDashboard("high ambiguity: " + String.format("%.2f", ambiguity));
                return;
            }
            double distMeters = tag.getBestCameraToTarget().getTranslation().getNorm();
            if (distMeters > VisionConstants.kMaxSingleTagDistanceMeters) {
                publishDashboard("single tag too far: " + String.format("%.1f", distMeters) + "m");
                return;
            }
        }

        var estimatedPose = m_poseEstimator.update(result);
        estimatedPose.ifPresent(est -> {
            m_latestEstimatedPose = est.estimatedPose.toPose2d();
            m_latestTimestamp     = est.timestampSeconds;
            m_hasTarget           = true;
        });

        publishDashboard(m_hasTarget ? "ok (" + hubTargets.size() + " tags)" : "estimator failed");
    }

    public boolean hasTarget()          { return m_hasTarget; }
    public Pose2d  getLatestPose()      { return m_latestEstimatedPose; }
    public double  getLatestTimestamp() { return m_latestTimestamp; }

    private void publishDashboard(String status) {
        SmartDashboard.putBoolean("Vision/HasTarget", m_hasTarget);
        SmartDashboard.putString("Vision/Status",     status);
        SmartDashboard.putNumber("Vision/Timestamp",  m_latestTimestamp);
        if (m_latestEstimatedPose != null) {
            SmartDashboard.putNumber("Vision/X", m_latestEstimatedPose.getX());
            SmartDashboard.putNumber("Vision/Y", m_latestEstimatedPose.getY());
        }
    }
}
