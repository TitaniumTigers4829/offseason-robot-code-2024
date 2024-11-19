// All praise 254 lib

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.vision.FiducialObservation;
import frc.robot.extras.vision.MegatagPoseEstimate;
import frc.robot.extras.vision.VisionFieldPoseEstimate;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionInterface visionInterface;
  private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

  public Vision(VisionInterface visionInterface) {
    // Initializing Fields
    this.visionInterface = visionInterface;
  }

  @Override
  public void periodic() {
    // Updates limelight inputs
    visionInterface.updateInputs(inputs);
    Logger.processInputs("Vision/", inputs);
  }

  // private void updateVision(
  //     MegatagPoseEstimate cameraMegatagPoseEstimate,
  //     MegatagPoseEstimate cameraMegatag2PoseEstimate,
  //     Limelight limelightName) {
  //   if (cameraMegatagPoseEstimate != null) {

  //     String logPreface = "Vision/" + visionInterface.getLimelightName(limelightName);
  //     var updateTimestamp = cameraMegatagPoseEstimate.timestampSeconds;
  //     boolean alreadyProcessedTimestamp =
  //         visionInterface.getTimeStampSeconds(limelightName) == updateTimestamp;
  //     if (!alreadyProcessedTimestamp && canSeeAprilTags(limelightName)) {

  //       Optional<VisionFieldPoseEstimate> megatagEstimate =
  //           processMegatagPoseEstimate(cameraMegatagPoseEstimate);
  //       Optional<VisionFieldPoseEstimate> megatag2Estimate =
  //           processMegatag2PoseEstimate(cameraMegatag2PoseEstimate, logPreface);

  //       boolean used_megatag = false;
  //       if (megatagEstimate.isPresent()) {
  //         if (shouldUseMegatag(
  //             cameraMegatagPoseEstimate, cameraFiducialObservations, isTurretCamera, logPreface)) {
  //           Logger.recordOutput(
  //               logPreface + "MegatagEstimate", megatagEstimate.get().getVisionRobotPoseMeters());
  //           state.updateMegatagEstimate(megatagEstimate.get());
  //           used_megatag = true;
  //         } else {
  //           if (megatagEstimate.isPresent()) {
  //             Logger.recordOutput(
  //                 logPreface + "MegatagEstimateRejected",
  //                 megatagEstimate.get().getVisionRobotPoseMeters());
  //           }
  //         }
  //       }

  //       if (megatag2Estimate.isPresent() && !used_megatag) {
  //         if (shouldUseMegatag2(cameraMegatag2PoseEstimate, logPreface)) {
  //           Logger.recordOutput(
  //               logPreface + "Megatag2Estimate", megatag2Estimate.get().getVisionRobotPoseMeters());
  //           state.updateMegatagEstimate(megatag2Estimate.get());
  //         } else {
  //           if (megatagEstimate.isPresent()) {
  //             Logger.recordOutput(
  //                 logPreface + "Megatag2EstimateRejected",
  //                 megatag2Estimate.get().getVisionRobotPoseMeters());
  //           }
  //         }
  //       }
  //     }
  //   }
  // }

  // private Optional<Pose2d> getFieldToRobotEstimate(
  //     MegatagPoseEstimate poseEstimate, boolean isTurretCamera) {
  //   var fieldToCamera = poseEstimate.fieldToCamera;
  //   if (fieldToCamera.getX() == 0.0) {
  //     return Optional.empty();
  //   }
  //   var cameraToTurretTransform = turretToCameraTransform.inverse();
  //   var fieldToTurretPose = fieldToCamera.plus(cameraToTurretTransform);
  //   var fieldToRobotEstimate = Pose2d.kZero;
  //   if (isTurretCamera) {
  //     var robotToTurretObservation = state.getRobotToTurret(poseEstimate.timestampSeconds);
  //     if (robotToTurretObservation.isEmpty()) {
  //       return Optional.empty();
  //     }
  //     var turretToRobot =
  //         MathHelpers.transform2dFromRotation(robotToTurretObservation.get().unaryMinus());
  //     fieldToRobotEstimate = fieldToTurretPose.plus(turretToRobot);
  //   } else {
  //     fieldToRobotEstimate = fieldToCamera.plus(turretToCameraTransform.inverse());
  //   }

  //   return Optional.of(fieldToRobotEstimate);
  // }

  // private Optional<VisionFieldPoseEstimate> processMegatag2PoseEstimate(
  //     MegatagPoseEstimate poseEstimate, String logPreface) {
  //   var loggedFieldToRobot = state.getFieldToRobot(poseEstimate.timestampSeconds);
  //   if (loggedFieldToRobot.isEmpty()) {
  //     return Optional.empty();
  //   }

  //   var maybeFieldToRobotEstimate = getFieldToRobotEstimate(poseEstimate, isTurretCamera);
  //   if (maybeFieldToRobotEstimate.isEmpty()) return Optional.empty();
  //   var fieldToRobotEstimate = maybeFieldToRobotEstimate.get();

  //   // distance from current pose to vision estimated pose
  //   double poseDifference =
  //       fieldToRobotEstimate
  //           .getTranslation()
  //           .getDistance(loggedFieldToRobot.get().getTranslation());

  //   var defaultSet = state.isRedAlliance() ? kTagsRedSpeaker : kTagsBlueSpeaker;
  //   Set<Integer> speakerTags = new HashSet<>(defaultSet);
  //   speakerTags.removeAll(
  //       Arrays.stream(poseEstimate.fiducialIds)
  //           .boxed()
  //           .collect(Collectors.toCollection(HashSet::new)));
  //   boolean seesSpeakerTags = speakerTags.size() < 2;

  //   double xyStds;
  //   if (poseEstimate.fiducialIds.length > 0) {
  //     // multiple targets detected
  //     if (poseEstimate.fiducialIds.length >= 2 && poseEstimate.avgTagArea > 0.1) {
  //       xyStds = 0.2;
  //     }
  //     // we detect at least one of our speaker tags and we're close to it.
  //     else if (seesSpeakerTags && poseEstimate.avgTagArea > 0.2) {
  //       xyStds = 0.5;
  //     }
  //     // 1 target with large area and close to estimated pose
  //     else if (poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
  //       xyStds = 0.5;
  //     }
  //     // 1 target farther away and estimated pose is close
  //     else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
  //       xyStds = 1.0;
  //     } else if (poseEstimate.fiducialIds.length > 1) {
  //       xyStds = 1.2;
  //     } else {
  //       xyStds = 2.0;
  //     }

  //     Logger.recordOutput(logPreface + "Megatag2StdDev", xyStds);
  //     Logger.recordOutput(logPreface + "Megatag2AvgTagArea", poseEstimate.avgTagArea);
  //     Logger.recordOutput(logPreface + "Megatag2PoseDifference", poseDifference);

  //     Matrix<N3, N1> visionMeasurementStdDevs =
  //         VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(50.0));
  //     fieldToRobotEstimate =
  //         new Pose2d(fieldToRobotEstimate.getTranslation(), loggedFieldToRobot.get().getRotation());
  //     return Optional.of(
  //         new VisionFieldPoseEstimate(
  //             fieldToRobotEstimate, poseEstimate.timestampSeconds, visionMeasurementStdDevs));
  //   }
  //   return Optional.empty();
  // }

  // private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(
  //     MegatagPoseEstimate poseEstimate, boolean isTurretCamera) {
  //   var loggedFieldToRobot = state.getFieldToRobot(poseEstimate.timestampSeconds);
  //   if (loggedFieldToRobot.isEmpty()) {
  //     return Optional.empty();
  //   }

  //   var maybeFieldToRobotEstimate = getFieldToRobotEstimate(poseEstimate, isTurretCamera);
  //   if (maybeFieldToRobotEstimate.isEmpty()) return Optional.empty();
  //   var fieldToRobotEstimate = maybeFieldToRobotEstimate.get();

  //   // distance from current pose to vision estimated pose
  //   double poseDifference =
  //       fieldToRobotEstimate
  //           .getTranslation()
  //           .getDistance(loggedFieldToRobot.get().getTranslation());

  //   if (poseEstimate.fiducialIds.length > 0) {
  //     double xyStds = 1.0;
  //     double degStds = 12;
  //     // multiple targets detected
  //     if (poseEstimate.fiducialIds.length >= 2) {
  //       xyStds = 0.5;
  //       degStds = 6;
  //     }
  //     // 1 target with large area and close to estimated pose
  //     else if (poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
  //       xyStds = 1.0;
  //       degStds = 12;
  //     }
  //     // 1 target farther away and estimated pose is close
  //     else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
  //       xyStds = 2.0;
  //       degStds = 30;
  //     }

  //     Matrix<N3, N1> visionMeasurementStdDevs =
  //         VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
  //     return Optional.of(
  //         new VisionFieldPoseEstimate(
  //             fieldToRobotEstimate, poseEstimate.timestampSeconds, visionMeasurementStdDevs));
  //   }
  //   return Optional.empty();
  // }

  // private Pose2d estimateFieldToRobot(
  //     Translation2d cameraToTarget,
  //     Pose3d fieldToTarget,
  //     Rotation2d robotToTurret,
  //     Rotation2d gyroAngle,
  //     Rotation2d cameraYawOffset,
  //     boolean isTurretCamera) {
  //   Transform2d cameraToTargetFixed =
  //       MathHelpers.transform2dFromTranslation(cameraToTarget.rotateBy(cameraYawOffset));
  //   Transform2d turretToTarget = state.getTurretToCamera(isTurretCamera).plus(cameraToTargetFixed);
  //   // In robot frame
  //   Transform2d robotToTarget = turretToTarget;
  //   if (isTurretCamera) {
  //     robotToTarget = MathHelpers.transform2dFromRotation(robotToTurret).plus(turretToTarget);
  //   }

  //   // In field frame
  //   Transform2d robotToTargetField =
  //       MathHelpers.transform2dFromTranslation(robotToTarget.getTranslation().rotateBy(gyroAngle));

  //   // In field frame
  //   Pose2d fieldToTarget2d =
  //       MathHelpers.pose2dFromTranslation(fieldToTarget.toPose2d().getTranslation());

  //   Pose2d fieldToRobot =
  //       fieldToTarget2d.transformBy(
  //           MathHelpers.transform2dFromTranslation(
  //               robotToTargetField.getTranslation().unaryMinus()));

  //   Pose2d fieldToRobotYawAdjusted = new Pose2d(fieldToRobot.getTranslation(), gyroAngle);
  //   return fieldToRobotYawAdjusted;
  // }

  // Add methods to support DriveCommand
  public int getNumberOfAprilTags(Limelight limelight) {
    return visionInterface.getNumberOfAprilTags(limelight);
  }

  public double getLimelightAprilTagDistance(Limelight limelight) {
    return visionInterface.getLimelightAprilTagDistance(limelight);
  }

  public double getTimeStampSeconds(Limelight limelight) {
    return visionInterface.getTimeStampSeconds(limelight);
  }

  public double getLatencySeconds(Limelight limelight) {
    return visionInterface.getLatencySeconds(limelight);
  }

  public void setHeadingInfo(double headingDegrees, double headingRateDegrees) {
    visionInterface.setHeadingInfo(headingDegrees, headingRateDegrees);
  }

  @AutoLogOutput(key = "Vision/Has Targets")
  public boolean canSeeAprilTags(Limelight limelight) {
    return visionInterface.canSeeAprilTags(limelight);
  }

  public Pose2d getPoseFromAprilTags(Limelight limelight) {
    return visionInterface.getPoseFromAprilTags(limelight);
  }

  public Pose2d getLastSeenPose() {
    return visionInterface.getLastSeenPose();
  }
}
