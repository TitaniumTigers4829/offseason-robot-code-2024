package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.extras.simulation.field.SimulatedField;
import frc.robot.extras.simulation.mechanismSim.swerve.SwerveDriveSimulation;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

// Simulate the vision system.
// Please see the following link for example code
// https://github.com/PhotonVision/photonvision/blob/2a6fa1b6ac81f239c59d724da5339f608897c510/photonlib-java-examples/swervedriveposeestsim/src/main/java/frc/robot/Vision.java
public class VisionIOSimPhoton extends VisionIOReal {
    private final PhotonCamera shooterCamera = new PhotonCamera("shooterCamera");
    private final PhotonCamera frontLeftCamera = new PhotonCamera("frontLeftCamera");
    private final PhotonCamera frontRightCamera = new PhotonCamera("frontRightCamera");
    PhotonCameraSim shooterCameraSim;
    PhotonCameraSim frontLeftCameraSim;
    PhotonCameraSim frontRightCameraSim;
    private final VisionSystemSim visionSim;
    private final Supplier<Pose2d> robotActualPoseInSimulationSupplier;

    private final int kResWidth = 1280;
    private final int kResHeight = 800;

    public VisionIOSimPhoton(Supplier<Pose2d> robotActualPoseInSimulationSupplier) throws IOException {
        this.robotActualPoseInSimulationSupplier = robotActualPoseInSimulationSupplier;
        // Create the vision system simulation which handles cameras and targets on the
        // field.
        visionSim = new VisionSystemSim("main");

        // Add all the AprilTags inside the tag layout as visible targets to this
        // simulated field.
        visionSim.addAprilTags(AprilTagFieldLayout.loadFromResource(VisionConstants.APRIL_TAG_FIELD.m_resourceFile));

        // Create simulated camera properties. These can be set to mimic your actual
        // camera.
        var turretProp = new SimCameraProperties();
        turretProp.setCalibration(kResWidth, kResHeight, Rotation2d.fromDegrees(97.7));
        turretProp.setCalibError(0.35, 0.10);
        turretProp.setFPS(15);
        turretProp.setAvgLatencyMs(20);
        turretProp.setLatencyStdDevMs(5);

        var elevatorProp = new SimCameraProperties();
        elevatorProp = turretProp.copy();

        // Create a PhotonCameraSim which will update the linked PhotonCamera's values
        // with visible
        // targets.
        // Instance variables
        shooterCameraSim = new PhotonCameraSim(shooterCamera, turretProp);
        frontLeftCameraSim = new PhotonCameraSim(frontLeftCamera, elevatorProp);
        frontRightCameraSim = new PhotonCameraSim(frontRightCamera, elevatorProp);

        // Add the simulated camera to view the targets on this simulated field.
        visionSim.addCamera(shooterCameraSim, new Transform3d());

        Transform3d robotToElevatorCamera = new Transform3d(
                new Translation3d(
                        0.0,
                        0.0,
                        0.0),
                new Rotation3d(0.0, 0.0, 0.0));
        visionSim.addCamera(frontLeftCameraSim, robotToElevatorCamera);


        // Enable the raw and processed streams. (http://localhost:1181 / 1182)
        shooterCameraSim.enableRawStream(true);
        shooterCameraSim.enableProcessedStream(true);
        frontLeftCameraSim.enableRawStream(true);
        frontLeftCameraSim.enableProcessedStream(true);
        frontRightCameraSim.enableRawStream(true);
        frontRightCameraSim.enableProcessedStream(true);

        // Enable drawing a wireframe visualization of the field to the camera streams.
        // This is extremely resource-intensive and is disabled by default.
        shooterCameraSim.enableDrawWireframe(true);
        frontLeftCameraSim.enableDrawWireframe(true);
        frontRightCameraSim.enableDrawWireframe(true);

    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Abuse the updateInputs periodic call to update the sim

        // Move the vision sim robot on the field
        if (robotActualPoseInSimulationSupplier.get() != null) {
            visionSim.update(robotActualPoseInSimulationSupplier.get());
            Logger.recordOutput("Vision/SimIO/updateSimPose", robotActualPoseInSimulationSupplier.get());
        }

        NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable(super.getLimelightName(0));
        NetworkTable frontLeftTable = NetworkTableInstance.getDefault().getTable(super.getLimelightName(1));
        NetworkTable frontRightTable = NetworkTableInstance.getDefault().getTable(super.getLimelightName(2));
        // Write to limelight table
        writeToTable(shooterCamera.getAllUnreadResults(), shooterTable);
        writeToTable(frontLeftCamera.getAllUnreadResults(), frontLeftTable);
        writeToTable(frontRightCamera.getAllUnreadResults(), frontRightTable);

        super.updateInputs(inputs);
    }

    private void writeToTable(List<PhotonPipelineResult> results, NetworkTable table) {
        //write to ll table
        for (PhotonPipelineResult result : results) {
        if (result.getMultiTagResult().isPresent()) {
            Transform3d best = result.getMultiTagResult().get().estimatedPose.best;
            Pose2d fieldToCamera = new Pose2d(best.getTranslation().toTranslation2d(),
                    best.getRotation().toRotation2d());
            List<Double> pose_data = new ArrayList<>(Arrays.asList(
                    best.getX(), // 0: X
                    best.getY(), // 1: Y
                    best.getZ(), // 2: Z,
                    0.0, // 3: roll
                    0.0, // 4: pitch
                    fieldToCamera.getRotation().getDegrees(), // 5: yaw
                    result.getTimestampSeconds(), // 6: latency ms,
                    (double) result.getMultiTagResult().get().fiducialIDsUsed.size(), // 7: tag count
                    0.0, // 8: tag span
                    0.0, // 9: tag dist
                    result.getBestTarget().getArea() // 10: tag area
            ));
            // Add RawFiducials
            // This is super inefficient but it's sim only, who cares.
            for (var target : result.targets) {
                pose_data.add((double) target.getFiducialId()); // 0: id
                pose_data.add(target.getYaw()); // 1: txnc
                pose_data.add(target.getPitch()); // 2: tync
                pose_data.add(0.0); // 3: ta
                pose_data.add(0.0); // 4: distToCamera
                pose_data.add(0.0); // 5: distToRobot
                pose_data.add(0.5); // 6: ambiguity
            }
        

            table.getEntry("botpose_wpiblue")
                    .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
            table.getEntry("botpose_orb_wpiblue")
                    .setDoubleArray(pose_data.stream().mapToDouble(Double::doubleValue).toArray());
        }

        table.getEntry("tv").setInteger(result.hasTargets() ? 1 : 0);
        // table.getEntry("cl").setDouble(result.getTimestampSeconds());
    }
}
}