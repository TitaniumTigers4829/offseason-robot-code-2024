package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.extras.vision.LimelightHelpers.PoseEstimate;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Arrays;
import java.util.Optional;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean cameraConnected = false;
        public double latency = 0.0;
        public double fiducialMarksID = 0.0;
        
        public int camerasAmount = 0;
        public int targetsCount = 0;
    }

    void updateInputs(VisionIOInputs inputs);

    String getLimelightName(int limelightNumber);

    double getLatencySeconds(int limelightNumber);
    
    double getTimeStampSeconds(int limelightNumber);

    boolean canSeeAprilTags(int limelightNumber);

    double getLimelightAprilTagDistance(int limelightNumber);

    int getNumberOfAprilTags(int limelightNumber);

    Pose2d getPoseFromAprilTags(int limelightNumber);

    void setHeadingInfo(double headingDegrees, double headingRateDegrees);
}