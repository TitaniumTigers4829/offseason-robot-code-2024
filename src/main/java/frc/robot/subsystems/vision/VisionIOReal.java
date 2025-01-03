package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.vision.LimelightHelpers;
import frc.robot.extras.vision.LimelightHelpers.PoseEstimate;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

public class VisionIOReal implements VisionIO {

  private Pose2d lastSeenPose = new Pose2d();
  private double headingDegrees = 0;
  private double headingRateDegreesPerSecond = 0;
  private final Map<Integer, AtomicBoolean> limelightThreads = new ConcurrentHashMap<>();
  private final ExecutorService executorService = Executors.newFixedThreadPool(3);

  /**
   * The pose estimates from the limelights in the following order {shooterLimelight,
   * frontLeftLimelight, frontRightLimelight}
   */
  private PoseEstimate[] limelightEstimates;

  public VisionIOReal() {
    limelightEstimates = new PoseEstimate[3];
    for (int limelightNumber = 0; limelightNumber < limelightEstimates.length; limelightNumber++) {
      limelightThreads.put(limelightNumber, new AtomicBoolean(true));
      limelightEstimates[limelightNumber] = new PoseEstimate();
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.camerasAmount = limelightEstimates.length;
    inputs.cameraConnected = true;

    for (int limelightNumber = 0; limelightNumber < limelightEstimates.length; limelightNumber++) {
      // Update camera connection status
      inputs.cameraConnected = true;

      // Add number of April tags seen by this limelight
      inputs.targetsCount += getNumberOfAprilTags(limelightNumber);

      // Add fiducial mark ID
      inputs.fiducialMarksID = LimelightHelpers.getFiducialID(getLimelightName(limelightNumber));

      // Add latency for this limelight
      inputs.latency += getLatencySeconds(limelightNumber) / 1000.0;
    }

    // Calculate average latency
    inputs.latency /= limelightEstimates.length;

    periodic();
  }

  /**
   * Checks if the specified limelight can fully see one or more April Tag.
   *
   * @param limelightNumber the number of the limelight
   * @return true if the limelight can fully see one or more April Tag
   */
  @Override
  public boolean canSeeAprilTags(int limelightNumber) {
    // First checks if it can see an april tag, then checks if it is fully in frame
    // Different Limelights have different FOVs
    if (getNumberOfAprilTags(limelightNumber) >= VisionConstants.MIN_APRIL_TAG_ID
        && getNumberOfAprilTags(limelightNumber) <= VisionConstants.MAX_APRIL_TAG_ID) {
      if (getLimelightName(limelightNumber).equals(VisionConstants.SHOOTER_LIMELIGHT_NAME)) {
        return Math.abs(LimelightHelpers.getTX(getLimelightName(limelightNumber)))
            <= VisionConstants.LL3G_FOV_MARGIN_OF_ERROR;
      } else {
        return Math.abs(LimelightHelpers.getTX(getLimelightName(limelightNumber)))
            <= VisionConstants.LL3_FOV_MARGIN_OF_ERROR;
      }
    }
    return false;
  }

  /**
   * Gets the JSON dump from the specified limelight and puts it into a PoseEstimate object, which
   * is then placed into its corresponding spot in the limelightEstimates array.
   *
   * @param limelightNumber the number of the limelight
   */
  public PoseEstimate enabledPoseUpdate(int limelightNumber) {
    if (canSeeAprilTags(limelightNumber) && isValidPoseEstimate(limelightNumber)) {
      if (isLargeDiscrepancyBetweenMegaTag1And2(limelightNumber)
          && getLimelightAprilTagDistance(limelightNumber)
              < VisionConstants.MEGA_TAG_2_DISTANCE_THRESHOLD) {
        return limelightEstimates[limelightNumber] = getMegaTag1PoseEstimate(limelightNumber);
      } else if (headingRateDegreesPerSecond < VisionConstants.MEGA_TAG_2_MAX_HEADING_RATE) {
        LimelightHelpers.SetRobotOrientation(
            getLimelightName(limelightNumber), headingDegrees, 0, 0, 0, 0, 0);
        return limelightEstimates[limelightNumber] = getMegaTag2PoseEstimate(limelightNumber);
      } else {
        return limelightEstimates[limelightNumber] = getMegaTag1PoseEstimate(limelightNumber);
      }
    }
    return limelightEstimates[limelightNumber] = new PoseEstimate();
  }

  /**
   * If the robot is not enabled, update the pose using MegaTag1 and after it is enabled, run {@link
   * #enabledPoseUpdate(int)}
   *
   * @param limelightNumber the number of the limelight
   */
  public void updatePoseEstimate(int limelightNumber) {
    limelightEstimates[limelightNumber] =
        DriverStation.isEnabled()
            ? enabledPoseUpdate(limelightNumber)
            : getMegaTag1PoseEstimate(limelightNumber);
  }

  /**
   * Checks if there is a large discrepancy between the MegaTag1 and MegaTag2 estimates.
   *
   * @param limelightNumber the number of the limelight
   * @return true if the discrepancy is larger than the defined threshold, false otherwise
   */
  public boolean isLargeDiscrepancyBetweenMegaTag1And2(int limelightNumber) {
    PoseEstimate megaTag1Estimate = getMegaTag1PoseEstimate(limelightNumber);
    PoseEstimate megaTag2Estimate = getMegaTag2PoseEstimate(limelightNumber);

    // Extract the positions of the two poses
    Translation2d megaTag1TranslationMeters = megaTag1Estimate.pose.getTranslation();
    Translation2d megaTag2TranslationMeters = megaTag2Estimate.pose.getTranslation();

    double megaTag1RotationDegrees = megaTag1Estimate.pose.getRotation().getDegrees();
    double megaTag2RotationDegrees = megaTag2Estimate.pose.getRotation().getDegrees();

    // Calculate the discrepancy between the two MegaTag translations in meters
    double megaTagTranslationDiscrepancyMeters =
        megaTag1TranslationMeters.getDistance(megaTag2TranslationMeters);
    double megaTagRotationDiscrepancyDegrees =
        Math.abs(megaTag1RotationDegrees - megaTag2RotationDegrees);

    // Define a threshold (meters) for what constitutes a "large" discrepancy
    // This value should be determined based on your testing
    double thresholdMeters = 0.5;
    double thresholdDegrees = 45;

    // Check if the discrepancy is larger than the threshold (meters)
    return megaTagTranslationDiscrepancyMeters > thresholdMeters
        || megaTagRotationDiscrepancyDegrees > thresholdDegrees;
  }

  /**
   * Gets the MegaTag1 pose of the robot calculated by specified limelight via any April Tags it
   * sees
   *
   * @param limelightNumber the number of the limelight
   * @return the MegaTag1 pose of the robot, if the limelight can't see any April Tags, it will
   *     return 0 for x, y, and theta
   */
  public PoseEstimate getMegaTag1PoseEstimate(int limelightNumber) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(getLimelightName(limelightNumber));
  }

  /**
   * Gets the MegaTag2 pose of the robot calculated by specified limelight via any April Tags it
   * sees
   *
   * @param limelightNumber the number of the limelight
   * @return the MegaTag2 pose of the robot, if the limelight can't see any April Tags, it will
   *     return 0 for x, y, and theta
   */
  public PoseEstimate getMegaTag2PoseEstimate(int limelightNumber) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getLimelightName(limelightNumber));
  }

  /**
   * Checks if the MT1 and MT2 pose estimate exists and whether it is within the field
   *
   * @param limelightNumber the number of the limelight
   * @return true if the pose estimate exists within the field and the pose estimate is not null
   */
  public boolean isValidPoseEstimate(int limelightNumber) {
    PoseEstimate megaTag1Estimate = getMegaTag1PoseEstimate(limelightNumber);
    PoseEstimate megaTag2Estimate = getMegaTag2PoseEstimate(limelightNumber);

    return LimelightHelpers.isValidPoseEstimate(megaTag1Estimate)
        && LimelightHelpers.isValidPoseEstimate(megaTag2Estimate)
        && isWithinFieldBounds(megaTag1Estimate, megaTag2Estimate);
  }

  /**
   * Checks whether the pose estimate for MT1 and MT2 is within the field
   *
   * @param megaTag1Estimate the MT1 pose estimate to check
   * @param megaTag2Estimate the MT2 pose estimate to check
   */
  private boolean isWithinFieldBounds(
      PoseEstimate megaTag1Estimate, PoseEstimate megaTag2Estimate) {
    return (megaTag1Estimate.pose.getX() > 0
            && megaTag1Estimate.pose.getX() <= FieldConstants.FIELD_WIDTH_METERS)
        && (megaTag1Estimate.pose.getY() > 0
            && megaTag1Estimate.pose.getY() <= FieldConstants.FIELD_WIDTH_METERS)
        && (megaTag2Estimate.pose.getX() > 0
            && megaTag2Estimate.pose.getX() <= FieldConstants.FIELD_WIDTH_METERS)
        && (megaTag2Estimate.pose.getY() > 0
            && megaTag2Estimate.pose.getY() <= FieldConstants.FIELD_WIDTH_METERS);
  }

  /**
   * Gets the pose of the robot calculated by specified limelight via any April Tags it sees
   *
   * @param limelightNumber the number of the limelight
   * @return the pose of the robot, if the limelight can't see any April Tags, it will return 0 for
   *     x, y, and theta
   */
  @Override
  public Pose2d getPoseFromAprilTags(int limelightNumber) {
    return limelightEstimates[limelightNumber].pose;
  }

  /** Returns how many april tags the limelight that is being used for pose estimation can see. */
  @Override
  public int getNumberOfAprilTags(int limelightNumber) {
    return limelightEstimates[limelightNumber].tagCount;
  }

  /**
   * Returns the timestamp for the vision measurement from the limelight that is being used for pose
   * estimation.
   */
  @Override
  public double getTimeStampSeconds(int limelightNumber) {
    return limelightEstimates[limelightNumber].timestampSeconds / 1000.0;
  }

  /**
   * Returns the latency in seconds of when the limelight that is being used for pose estimation
   * calculated the robot's pose. It adds the pipeline latency, capture latency, and json parsing
   * latency.
   */
  @Override
  public double getLatencySeconds(int limelightNumber) {
    return (limelightEstimates[limelightNumber].latency) / 1000.0;
  }

  /** Gets the pose calculated the last time a limelight saw an April Tag */
  public Pose2d getLastSeenPose() {
    return lastSeenPose;
  }

  /**
   * Gets the average distance between the specified limelight and the April Tags it sees
   *
   * @param limelightNumber the number of the limelight
   * @return the average distance between the robot and the April Tag(s) in meters
   */
  public double getLimelightAprilTagDistance(int limelightNumber) {
    if (canSeeAprilTags(limelightNumber)) {
      return limelightEstimates[limelightNumber].avgTagDist;
    }
    // To be safe returns a big distance from the april tags if it can't see any
    return Double.MAX_VALUE;
  }

  /**
   * Sets the heading and heading rate of the robot, this is used for deciding between MegaTag 1 and
   * 2 for pose estimation.
   *
   * @param headingDegrees the angle the robot is facing in degrees (0 degrees facing the red
   *     alliance)
   * @param headingRateDegrees the rate the robot is rotating, CCW positive
   */
  public void setHeadingInfo(double headingDegrees, double headingRateDegrees) {
    this.headingDegrees = headingDegrees;
    this.headingRateDegreesPerSecond = headingRateDegrees;
  }

  /**
   * Gets the limelight name associated with the specified limelight number/index
   *
   * @param limelightNumber the limelight number
   * @return 0 = limelight-shooter, 1 = limelight-left, 2 = limelight-right
   */
  public String getLimelightName(int limelightNumber) {
    return switch (limelightNumber) {
      case 0 -> VisionConstants.SHOOTER_LIMELIGHT_NAME;
      case 1 -> VisionConstants.FRONT_LEFT_LIMELIGHT_NAME;
      case 2 -> VisionConstants.FRONT_RIGHT_LIMELIGHT_NAME;
      default ->
          throw new IllegalArgumentException("You entered a number for a non-existent limelight");
    };
  }

  /**
   * This checks is there is new pose detected by a limelight, and if so, updates the pose estimate
   *
   * @param limelightNumber the limelight number
   */
  public void checkAndUpdatePose(int limelightNumber) {
    double last_TX = 0;
    double last_TY = 0;

    // Syncronization block to ensure thread safety during the critical section where pose
    // information is read and compared.
    // This helps prevent race conditions, where one limelight may be updating an object that
    // another limelight is reading.
    // A race condition could cause unpredictable things to happen. Such as causing a limelight to
    // be unable to reference an
    // object, as its reference was modified earlier.
    synchronized (this) {
      try {
        double current_TX = LimelightHelpers.getTX(getLimelightName(limelightNumber));
        double current_TY = LimelightHelpers.getTY(getLimelightName(limelightNumber));

        // This checks if the limelight reading is new. The reasoning being that if the TX and TY
        // are EXACTLY the same, it hasn't updated yet with a new reading. We are doing it this way,
        // because to get the timestamp of the reading, you need to parse the JSON dump which can be
        // very demanding whereas this only has to get the Network Table entries for TX and TY.
        if (current_TX != last_TX || current_TY != last_TY) {
          updatePoseEstimate(limelightNumber);
          limelightThreads.computeIfPresent(
              limelightNumber, (key, value) -> new AtomicBoolean(true));
          // This is to keep track of the last valid pose calculated by the limelights
          // it is used when the driver resets the robot odometry to the limelight calculated
          // position
          if (canSeeAprilTags(limelightNumber)) {
            lastSeenPose = getMegaTag1PoseEstimate(limelightNumber).pose;
          }
        } else {
          // Retrieve the AtomicBoolean for the given limelight number
          AtomicBoolean isThreadRunning =
              limelightThreads.getOrDefault(limelightNumber, new AtomicBoolean());
          // Only stop the thread if it's currently running
          if (isThreadRunning.get()) {
            // stop the thread for the specified limelight
            stopThread(limelightNumber);
          }
        }
        last_TX = current_TX;
        last_TY = current_TY;
      } catch (Exception e) {
        System.err.println(
            "Error communicating with the: "
                + getLimelightName(limelightNumber)
                + ": "
                + e.getMessage());
      }
    }
  }

  /**
   * Starts a separate thread dedicated to updating the pose estimate for a specified limelight.
   * This approach is adopted to prevent loop overruns that would occur if we attempted to parse the
   * JSON dump for each limelight sequentially within a single scheduler loop.
   *
   * <p>To achieve efficient and safe parallel execution, an ExecutorService is utilized to manage
   * the lifecycle of these threads.
   *
   * <p>Each thread continuously runs the {@link #checkAndUpdatePose(int)} method as long as the
   * corresponding limelight's thread is marked as "running". This ensures that pose estimates are
   * updated in real-time, leveraging the parallel processing capabilities of the executor service.
   *
   * @param limelightNumber the limelight number
   */
  public void visionThread(int limelightNumber) {

    executorService.submit(
        () -> {
          try {
            // while (limelightThreads.get(limelightNumber).get()) {
            checkAndUpdatePose(limelightNumber);
            // }
          } catch (Exception e) {
            System.err.println(
                "Error executing task for the: "
                    + getLimelightName(limelightNumber)
                    + ": "
                    + e.getMessage());
          }
        });
  }

  /**
   * Sets the AtomicBoolean 'runningThreads' to false for the specified limelight. Stops the thread
   * for the specified limelight.
   *
   * @param limelightNumber the limelight number
   */
  public void stopThread(int limelightNumber) {
    try {
      // Since we can't see an April Tag, set the estimate for the specified limelight to an empty
      // PoseEstimate()
      limelightEstimates[limelightNumber] = new PoseEstimate();
      limelightThreads.get(limelightNumber).set(false);
    } catch (Exception e) {
      System.err.println(
          "Error stopping thread for the: "
              + getLimelightName(limelightNumber)
              + ": "
              + e.getMessage());
    }
  }

  /** Shuts down all the threads. */
  public void endAllThreads() {
    // Properly shut down the executor service when the subsystem ends
    executorService.shutdown(); // Prevents new tasks from being submitted
    try {
      // Wait for existing tasks to finish
      if (!executorService.awaitTermination(5, TimeUnit.SECONDS)) {
        executorService.shutdownNow();
        // Wait a bit longer for tasks to respond to being cancelled
        if (!executorService.awaitTermination(5, TimeUnit.SECONDS))
          System.err.println("ExecutorService did not terminate");
      }
    } catch (InterruptedException e) {
      // (Re-)Cancel if current thread also interrupted
      executorService.shutdownNow();
      // Preserve interrupt status
      Thread.currentThread().interrupt();
    }
  }

  // Override periodic method to start the vision threads at the beginning of each subsystem tick
  public void periodic() {
    visionThread(VisionConstants.SHOOTER_LIMELIGHT_NUMBER);
    visionThread(VisionConstants.FRONT_LEFT_LIMELIGHT_NUMBER);
    visionThread(VisionConstants.FRONT_RIGHT_LIMELIGHT_NUMBER);
    SmartDashboard.putNumber("april tag dist", getLimelightAprilTagDistance(0));
    SmartDashboard.putString("shooter ll odom", getPoseFromAprilTags(0).toString());
    SmartDashboard.putString("left ll odom", getPoseFromAprilTags(1).toString());

    SmartDashboard.putString("right ll odom", getPoseFromAprilTags(2).toString());
  }
}
