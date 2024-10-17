package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO visionIO;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    /**
     * Position of camera on the robot. This is calculated through the horizontal distance (in meters)
     * from the fiducial to the lens of the camera.
     */
    private Pose2d cameraToAprilTagPose = new Pose2d();

    /** Horizontal Distance (in meters) from the Fiducial marker to the Lens of the camera. */
    private double horizontalDistanceToTargetMeters = 0;

    /**
     * Contains information about the mounting of the limelight on the robot. This is required when
     * calculating the horizontalDistanceToTargetMeters.
     */
    private final LimelightConfiguration limelightConfiguration;

    public Vision(VisionIO visionIO, LimelightConfiguration limelightConfiguration) {
        // Initializing Fields
        this.visionIO = visionIO;
        this.limelightConfiguration = limelightConfiguration;
    }

    @Override
    public void periodic() {
        // Updates limelight inputs
        visionIO.updateInputs(inputs);
        Logger.processInputs(limelightConfiguration.Name, inputs);

        // Update camera position based on April Tag pose
        updateCameraPosition();
    }

    private void updateCameraPosition() {
        Pose2d aprilTagPose = visionIO.getPoseFromAprilTags(0); // Assuming we're using the shooter limelight
        if (aprilTagPose != null) {
            cameraToAprilTagPose = Pose2d.fromTranslation(
                Translation2d.fromFeet(aprilTagPose.getX(), aprilTagPose.getY()),
                Rotation2d.fromDegrees(aprilTagPose.getRotation().getDegrees())
            );
            
            // Calculate horizontal distance to target
            horizontalDistanceToTargetMeters = calculateHorizontalDistanceToTarget();
        }
    }

    private double calculateHorizontalDistanceToTarget() {
        // Implementation depends on how you want to calculate the horizontal distance
        // You might need to use the limelight configuration and the camera position
        return 0; // Placeholder value
    }

    @AutoLogOutput(key = "Vision/Horizontal Distance to Target")
    public double getHorizontalDistanceToTargetMeters() {
        return horizontalDistanceToTargetMeters;
    }

    @AutoLogOutput(key = "Vision/Has Targets")
    public boolean hasTargets() {
        return visionIO.canSeeAprilTags(0); // Assuming we're checking the shooter limelight
    }

    @AutoLogOutput(key = "Vision/Yaw (tx)")
    public double getYawRadians() {
        return visionIO.getLimelightName(0).equals(visionIO.getLimelightName(0)) ? 
               visionIO.getLimelightHelpers().getTX(visionIO.getLimelightName(0)) : 
               0;
    }

    @AutoLogOutput(key = "Vision/Pitch (ty)")
    public double getPitchRadians() {
        return visionIO.getLimelightHelpers().getTY(visionIO.getLimelightName(0));
    }

    public void setLeds(boolean on) {
        visionIO.setLeds(on);
    }

    // Add methods to access other limelight estimates
    public Pose2d getShooterLimelightPose() {
        return visionIO.getPoseFromAprilTags(0);
    }

    public Pose2d getFrontLeftLimelightPose() {
        return visionIO.getPoseFromAprilTags(1);
    }

    public Pose2d getFrontRightLimelightPose() {
        return visionIO.getPoseFromAprilTags(2);
    }
}