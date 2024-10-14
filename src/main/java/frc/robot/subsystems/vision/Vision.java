package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.debug.Alert;
import frc.robot.subsystems.swerve.HolonomicDriveSubsystem;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.util.List;
import java.util.Optional;
import java.util.function.Function;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIO.VisionIOInputs inputs;

    private final AprilTagFieldLayout fieldLayout;

    private final PhotonPoseEstimator poseEstimator;
    PoseStrategy poseStrategy;

    private final HolonomicDriveSubsystem driveSubsystem;
    private final Alert[] camerasDisconnectedAlerts;
    public Vision(VisionIO io, List<PhotonCameraProperties> camerasProperties, HolonomicDriveSubsystem driveSubsystem) {
        super("Vision");
        this.io = io;
        this.inputs = new VisionIO.VisionIOInputs(camerasProperties.size());
        this.camerasDisconnectedAlerts = new Alert[camerasProperties.size()];
        for (int i = 0; i < camerasProperties.size(); i++) {
            this.camerasDisconnectedAlerts[i] = new Alert(
                    "Photon Camera " + i + " '" + camerasProperties.get(i).name + "' disconnected",
                    Alert.AlertType.WARNING
            );
            this.camerasDisconnectedAlerts[i].setActivated(false);
        }
        this.poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_RIO;
        this.fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        this.poseEstimator = new PhotonPoseEstimator(fieldLayout, poseStrategy, );

        // this.multiTagPoseEstimator = new MapleMultiTagPoseEstimator(
        //         fieldLayout,
        //         new CameraHeightAndPitchRollAngleFilter(),
        //         camerasProperties
        // );
        this.driveSubsystem = driveSubsystem;
    }

    private Optional<RobotPoseEstimationResult> result = Optional.empty();
    @Override
    public void periodic(double dt, boolean enabled) {
        io.updateInputs(inputs);
        Logger.processInputs(APRIL_TAGS_VISION_PATH + "Inputs", inputs);

        for (int i = 0; i < inputs.camerasInputs.length; i++)
            this.camerasDisconnectedAlerts[i].setActivated(!inputs.camerasInputs[i].cameraConnected);

        result = multiTagPoseEstimator.estimateRobotPose(inputs.camerasInputs, driveSubsystem.getPose());
        result = discardResultIfOverThreshold(result);
        result.ifPresent(robotPoseEstimationResult -> driveSubsystem.addVisionMeasurement(
                robotPoseEstimationResult.pointEstimation,
                getResultsTimeStamp(),
                robotPoseEstimationResult.estimationStandardError
        ));

        Logger.recordOutput(
                APRIL_TAGS_VISION_PATH + "Results/Standard Error",
                result.map(printStandardError).orElse(null)
        );

        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Results/Estimated Pose", displayVisionPointEstimateResult(result));
        SmartDashboard.putBoolean("Vision Result Trustable", result.isPresent());
        Logger.recordOutput(APRIL_TAGS_VISION_PATH + "Results/Presented", result.isPresent());
    }

    private Optional<RobotPoseEstimationResult> discardResultIfOverThreshold(Optional<RobotPoseEstimationResult> result) {
        if (result.isEmpty())
            return result;

        double standardDeviationX = result.get().translationXStandardDeviationMeters,
                standardDeviationY = result.get().translationYStandardDeviationMeters,
                standardDeviationTheta = result.get().rotationalStandardDeviationRadians;
        /* don't calibrate odometry if translation error is not inside range */
        if (standardDeviationX > TRANSLATIONAL_STANDARD_ERROR_THRESHOLD || standardDeviationY > TRANSLATIONAL_STANDARD_ERROR_THRESHOLD)
            return Optional.empty();
        /* don't calibrate gyro if rotation error is not inside range */
        if (standardDeviationTheta > ROTATIONAL_STANDARD_ERROR_THRESHOLD) {
            standardDeviationTheta = Double.POSITIVE_INFINITY;
            if (Math.abs(driveSubsystem.getPose().getRotation()
                    .minus(result.get().pointEstimation.getRotation())
                    .getRadians()) > ROTATIONAL_ERROR_WITH_GYRO_DISCARD_RESULT
            ) return Optional.empty();
        }

        return Optional.of(new RobotPoseEstimationResult(
                result.get().pointEstimation,
                standardDeviationX,
                standardDeviationY,
                standardDeviationTheta
        ));
    }

    private Pose2d displayVisionPointEstimateResult(Optional<RobotPoseEstimationResult> result) {
        if (result.isEmpty()) return null;

        if (Double.isInfinite(result.get().rotationalStandardDeviationRadians))
            return new Pose2d(result.get().pointEstimation.getTranslation(), driveSubsystem.getFacing());
        return result.get().pointEstimation;
    }

    private double getResultsTimeStamp() {
        return inputs.inputsFetchedRealTimeStampSeconds - getResultsAverageLatencySeconds(inputs.camerasInputs);
    }
    private static double getResultsAverageLatencySeconds(AprilTagVisionIO.CameraInputs[] camerasInputs) {
        if (camerasInputs.length == 0)
            return 0;
        double totalLatencySeconds = 0;
        for (AprilTagVisionIO.CameraInputs cameraInputs:camerasInputs)
            totalLatencySeconds += cameraInputs.resultsDelaySeconds;

        return totalLatencySeconds / camerasInputs.length;
    }

    private static final Function<RobotPoseEstimationResult, String> printStandardError = result ->
            String.format(
                    "Standard Error : { \n"
                            + "Translation X (Meters) %.2f, \n"
                            + "Translation Y (Meters) %.2f, \n"
                            + "Rotation Theta (Degrees) %.2f, \n }",
                    result.estimationStandardError.get(0, 0),
                    result.estimationStandardError.get(1, 0),
                    Math.toDegrees(result.estimationStandardError.get(2, 0))
            );
}