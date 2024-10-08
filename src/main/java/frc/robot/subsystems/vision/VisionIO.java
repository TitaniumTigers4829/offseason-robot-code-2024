package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
 
public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        Pose3d[] pose = new Pose3d[] {};
        double[] timestamp = new double[] {};
        int[] tags = new int[] {};
        boolean CamHastargets = false;
        double Camyaw = 0;
        public double lastTimeStamp;
        public double horizontalAngleRadians;
        public double verticalAngleRadians;
        public boolean hasTargets;
        public double[] botpose;
        public int tagId;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setReferencePose(Pose2d reference) {}

    public default double getDistanceToTag(int tag) { 
        return 0.0;
    }
} 

