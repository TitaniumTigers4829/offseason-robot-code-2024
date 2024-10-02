package frc.robot.subsystems.vision;


import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhoton implements VisionIO {
    PhotonPipelineResult result;

    PhotonCamera camera;

    PhotonPoseEstimator poseEstimator;

    Pose2d speakerPosition;
    double distanceToTarget;

    Pose3d tagPose;

    AprilTagFieldLayout aprilTagFieldLayout;

    public VisionIOPhoton(int camIndex) throws IOException {
        camera = new PhotonCamera("");
         
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));

        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void setReferencePose(Pose2d reference) {
        poseEstimator.setReferencePose(reference);
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {
        poseEstimator.update().ifPresentOrElse((pose) -> {
            inputs.pose = new Pose3d[] {pose.estimatedPose};
            inputs.timestamp = new double[] {pose.timestampSeconds};
            inputs.tags = pose.targetsUsed.stream().mapToInt((target) -> target.getFiducialId()).toArray();
        }, () -> {
            inputs.pose = new Pose3d[] {};
            inputs.timestamp = new double[] {};
            inputs.tags = new int[] {};
        });
    }
}