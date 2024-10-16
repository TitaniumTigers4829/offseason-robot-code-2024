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
  }

  @AutoLogOutput(key = "Vision/Horizontal Distance to Target")
  public double getHorizontalDistanceToTargetMeters() {
    return horizontalDistanceToTargetMeters;
  }

  @AutoLogOutput(key = "Vision/Has Targets")
  public boolean hasTargets() {
    return getPitchRadians() != 0.00;
    //        return inputs.hasTargets;
  }

  @AutoLogOutput(key = "Vision/Yaw (tx)")
  public double getYawRadians() {
    return inputs.horizontalAngleRadians;
  }

  @AutoLogOutput(key = "Vision/Pitch (ty)")
  public double getPitchRadians() {
    return inputs.verticalAngleRadians;
  }

  public void setLeds(boolean on) {
    visionIO.setLeds(on);
  }
}
