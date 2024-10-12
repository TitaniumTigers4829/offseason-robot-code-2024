// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/** Add your docs here. */
SimCameraProperties cameraProp = new SimCameraProperties();
public class VisionIOSim {
    VisionIOSim visionSim = new VisionIOSim();
    TargetModel targetModel = new getModel(TargetModel.kAprilTag36h11);
    Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
    // The given target model at the given pose
    VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

    
    }
visionSim.addVisionTargets(visionTarget);
visionSim.addAprilTags(tagLayout);