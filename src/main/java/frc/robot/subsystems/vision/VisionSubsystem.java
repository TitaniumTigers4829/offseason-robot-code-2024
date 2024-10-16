package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  PhotonCamera camera = new PhotonCamera("photonvision");

  private Pose3d estimatedPose = new Pose3d();
  PhotonPipelineResult result = camera.getLatestResult();
  private PhotonTrackedTarget target = result.getBestTarget();
  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  Transform3d cameraToTarget = new Transform3d();
  Pose3d fieldToTarget;
  Transform3d cameraToRobot;

  public VisionSubsystem() {}

  public boolean canSeeAprilTags() {
    return camera.getLatestResult().hasTargets();
    // checks to see if there are targets in the vicinity of the vision based on the current view
  }

  public Pose3d getPoseFromCamera() {
    return PhotonUtils.estimateFieldToRobotAprilTag(
        target.getBestCameraToTarget(),
        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(),
        cameraToRobot);
  }

  public double getDistanceFromClosestVisibleAprilTag() {
    return PhotonUtils.calculateDistanceToTargetMeters(
        0, 0, 0, Units.degreesToRadians(result.getBestTarget().getPitch()));
  }

  public boolean isValidPoseEstitmate(Pose2d pose) {
    return false;
  }

  public int getNumberOfAprilTags() {
    return 0;
  }

  public double getLatencyMiliseconds() {
    return 0;
  }

  public void checkAndUpdatePose() {
    camera.getLatestResult();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
 // VERY MUCH SO INCOMPLETE
 // TODO: Fill in values
 // TODO: idk more coding stuff that's in depth
