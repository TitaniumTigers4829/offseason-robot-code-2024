package frc.robot.subsystems.vision;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionIOSim implements VisionIO {
    VisionSystemSim visionSim = new VisionSystemSim("main");

    

    

    // A 0.5 x 0.25 meter rectangular target
    TargetModel targetModel = new TargetModel(0.5, 0.25);

    // The pose of where the target is on the field.
    // Its rotation determines where "forward" or the target x-axis points.
    // Let's say this target is flat against the far wall center, facing the blue driver stations.
    Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));

    // The given target model at the given pose
    VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);

    // The layout of AprilTags which we want to add to the vision system
    AprilTagFieldLayout tagLayout;
PhotonCameraSim cameraSim;
    
    
    public VisionIOSim() throws IOException {
        tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        SimCameraProperties cameraProp = new SimCameraProperties();

    // The PhotonCamera used in the real robot code.
PhotonCamera camera = new PhotonCamera("cameraName");

// The simulation of this camera. Its values used in real robot code will be updated.
 cameraSim = new PhotonCameraSim(camera, cameraProp);

        visionSim.addAprilTags(tagLayout);
        // Add this vision target to the vision system simulation to make it visible
        visionSim.addVisionTargets(visionTarget);
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);
        // TODO: NEEDS TO BE ADJUSTED WITH ACTUAL CAMERA VALUES

        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        // Add this camera to the vision system simulation with the given robot-to-camera transform.
        visionSim.addCamera(cameraSim, robotToCamera);
        
        visionSim.getDebugField();
        // Enable the raw and processed streams. These are enabled by default.
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        // Enable drawing a wireframe visualization of the field to the camera streams.
        // This is extremely resource-intensive and is disabled by default.
        cameraSim.enableDrawWireframe(true);
    }
    public void simulationPeriodic() {
        visionSim.update(cameraSim.getMaxSightRangeMeters());
    }
}
