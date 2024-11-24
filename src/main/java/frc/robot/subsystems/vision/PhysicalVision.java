package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.extras.util.GeomUtil;
import frc.robot.extras.vision.LimelightHelpers;
import frc.robot.extras.vision.LimelightHelpers.PoseEstimate;
import frc.robot.extras.vision.MegatagPoseEstimate;
import frc.robot.subsystems.vision.VisionConstants.Limelight;
import frc.robot.subsystems.vision.VisionInterface.VisionInputs;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

public class PhysicalVision implements VisionInterface {

  private Pose2d lastSeenPose = new Pose2d();
  private double headingDegrees = 0;
  private double headingRateDegreesPerSecond = 0;
  private final ConcurrentHashMap<Limelight, AtomicReference<VisionInputs>> limelightThreads =
      new ConcurrentHashMap<>();
  private final ExecutorService executorService = Executors.newFixedThreadPool(3);
  private final AtomicReference<VisionInputs> latestInputs = new AtomicReference<>();
  private final ThreadManager threadManager = new ThreadManager(3);

  /**
   * The pose estimates from the limelights in the following order {shooterLimelight,
   * frontLeftLimelight, frontRightLimelight}
   */
  private PoseEstimate[] limelightEstimates =
      new PoseEstimate[] {new PoseEstimate(), new PoseEstimate(), new PoseEstimate()};

  public PhysicalVision() {
    for (Limelight limelight : Limelight.values()) {
      VisionInputs inputs = new VisionInputs(); // Create vision inputs for each Limelight
      limelightThreads.put(limelight, new AtomicReference<>(inputs));

      // Start a vision input task for each Limelight
      threadManager.startVisionInputTask(
          limelight.getName(), inputs, () -> visionThreadTask(limelight, inputs));
    }
  }

  @Override
  public void updateInputs(VisionInputs inputs) {
    limelightThreads.forEach(
        (limelight, inputRef) -> {
          VisionInputs limelightInputs = inputRef.get();
          // Combine inputs into the main inputs object
          switch (limelight) {
            case SHOOTER -> {
              inputs.isShooterLimelightConnected = limelightInputs.isShooterLimelightConnected;
              inputs.shooterMegaTag1Pose = limelightInputs.shooterMegaTag1Pose;
              inputs.shooterLatency = limelightInputs.shooterLatency;
              inputs.shooterMegaTag2Pose = limelightInputs.shooterMegaTag2Pose;
            }
            case FRONT_LEFT -> {
              inputs.isFrontLeftLimelightConnected = limelightInputs.isFrontLeftLimelightConnected;
              inputs.frontLeftMegaTag1Pose = limelightInputs.frontLeftMegaTag1Pose;
              inputs.frontLeftLatency = limelightInputs.frontLeftLatency;
              inputs.frontLeftMegaTag2Pose = limelightInputs.frontLeftMegaTag2Pose;
            }
            case FRONT_RIGHT -> {
              inputs.isFrontRightLimelightConnected =
                  limelightInputs.isFrontRightLimelightConnected;
              inputs.frontRightMegaTag1Pose = limelightInputs.frontRightMegaTag1Pose;
              inputs.frontRightLatency = limelightInputs.frontRightLatency;
              inputs.frontRightMegaTag2Pose = limelightInputs.frontRightMegaTag2Pose;
            }
          }
        });
    latestInputs.set(inputs);
  }

  /**
   * Checks if the specified limelight can fully see one or more April Tag.
   *
   * @param limelight a limelight (SHOOTER, FRONT_LEFT, FRONT_RIGHT).
   * @return true if the limelight can fully see one or more April Tag
   */
  @Override
  public boolean canSeeAprilTags(Limelight limelight) {
    // First checks if it can see an april tag, then checks if it is fully in frame
    // Different Limelights have different FOVs
    if (getNumberOfAprilTags(limelight) > 0
        && getNumberOfAprilTags(limelight) <= VisionConstants.APRIL_TAG_POSITIONS.length) {
      if (limelight.getName().equals(VisionConstants.SHOOTER_LIMELIGHT_NAME)) {
        return Math.abs(LimelightHelpers.getTX(limelight.getName()))
            <= VisionConstants.LL3G_FOV_MARGIN_OF_ERROR;
      } else {
        return Math.abs(LimelightHelpers.getTX(limelight.getName()))
            <= VisionConstants.LL3_FOV_MARGIN_OF_ERROR;
      }
    }
    return false;
  }

  /**
   * Gets the JSON dump from the specified limelight and puts it into a PoseEstimate object, which
   * is then placed into its corresponding spot in the limelightEstimates array.
   *
   * @param limelight a limelight (SHOOTER, FRONT_LEFT, FRONT_RIGHT).
   */
  public PoseEstimate enabledPoseUpdate(Limelight limelight) {
    if (canSeeAprilTags(limelight) && isValidPoseEstimate(limelight)) {
      if (isLargeDiscrepancyBetweenMegaTag1And2(limelight)
          && getLimelightAprilTagDistance(limelight)
              < VisionConstants.MEGA_TAG_2_DISTANCE_THRESHOLD) {
        return limelightEstimates[limelight.getId()] = getMegaTag1PoseEstimate(limelight);
      } else if (headingRateDegreesPerSecond < VisionConstants.MEGA_TAG_2_MAX_HEADING_RATE) {
        LimelightHelpers.SetRobotOrientation(limelight.getName(), headingDegrees, 0, 0, 0, 0, 0);
        return limelightEstimates[limelight.getId()] = getMegaTag2PoseEstimate(limelight);
      } else {
        return limelightEstimates[limelight.getId()] = getMegaTag1PoseEstimate(limelight);
      }
    }
    return limelightEstimates[limelight.getId()] = new PoseEstimate();
  }

  /**
   * If the robot is not enabled, update the pose using MegaTag1 and after it is enabled, run {@link
   * #enabledPoseUpdate(int)}
   *
   * @param limelight a limelight (SHOOTER, FRONT_LEFT, FRONT_RIGHT).
   */
  public void updatePoseEstimate(Limelight limelight) {
    limelightEstimates[limelight.getId()] =
        DriverStation.isEnabled()
            ? enabledPoseUpdate(limelight)
            : getMegaTag1PoseEstimate(limelight);
  }

  /**
   * Checks if there is a large discrepancy between the MegaTag1 and MegaTag2 estimates.
   *
   * @param limelight a limelight (SHOOTER, FRONT_LEFT, FRONT_RIGHT).
   * @return true if the discrepancy is larger than the defined threshold, false otherwise
   */
  public boolean isLargeDiscrepancyBetweenMegaTag1And2(Limelight limelight) {
    PoseEstimate megatag1Estimate = getMegaTag1PoseEstimate(limelight);
    PoseEstimate megatag2Estimate = getMegaTag2PoseEstimate(limelight);

    return !GeomUtil.isTranslationWithinThreshold(
            megatag1Estimate.pose.getTranslation(),
            megatag2Estimate.pose.getTranslation(),
            VisionConstants.MEGA_TAG_TRANSLATION_DISCREPANCY_THRESHOLD)
        || !GeomUtil.isRotationWithinThreshold(
            megatag1Estimate.pose.getRotation().getDegrees(),
            megatag2Estimate.pose.getRotation().getDegrees(),
            VisionConstants.MEGA_TAG_ROTATION_DISCREPANCY_THREASHOLD);
  }

  /**
   * Gets the MegaTag1 pose of the robot calculated by specified limelight via any April Tags it
   * sees
   *
   * @param limelight a limelight (SHOOTER, FRONT_LEFT, FRONT_RIGHT).
   * @return the MegaTag1 pose of the robot, if the limelight can't see any April Tags, it will
   *     return 0 for x, y, and theta
   */
  public PoseEstimate getMegaTag1PoseEstimate(Limelight limelight) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight.getName());
  }

  /**
   * Gets the MegaTag2 pose of the robot calculated by specified limelight via any April Tags it
   * sees
   *
   * @param limelight a limelight (SHOOTER, FRONT_LEFT, FRONT_RIGHT).
   * @return the MegaTag2 pose of the robot, if the limelight can't see any April Tags, it will
   *     return 0 for x, y, and theta
   */
  public PoseEstimate getMegaTag2PoseEstimate(Limelight limelight) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight.getName());
  }

  /**
   * Checks if the MT1 and MT2 pose estimate exists and whether it is within the field
   *
   * @param limelight a limelight (SHOOTER, FRONT_LEFT, FRONT_RIGHT).
   * @return true if the pose estimate exists within the field and the pose estimate is not null
   */
  public boolean isValidPoseEstimate(Limelight limelight) {
    PoseEstimate megaTag1Estimate = getMegaTag1PoseEstimate(limelight);
    PoseEstimate megaTag2Estimate = getMegaTag2PoseEstimate(limelight);

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
   * @param limelight a limelight (SHOOTER, FRONT_LEFT, FRONT_RIGHT).
   * @return the pose of the robot, if the limelight can't see any April Tags, it will return 0 for
   *     x, y, and theta
   */
  @Override
  public Pose2d getPoseFromAprilTags(Limelight limelight) {
    return limelightEstimates[limelight.getId()].pose;
  }

  public Pose2d getAprilTagPositionToLimelight(Limelight limelight) {
    return LimelightHelpers.getTargetPose_CameraSpace(limelight.getName());
  }

  public Pose2d getAprilTagPositionToRobot(Limelight limelight) {
    return LimelightHelpers.getTargetPose_RobotSpace(limelight.getName());
  }

  /** Returns how many april tags the limelight that is being used for pose estimation can see. */
  @Override
  public int getNumberOfAprilTags(Limelight limelight) {
    return limelightEstimates[limelight.getId()].tagCount;
  }

  /**
   * Returns the timestamp for the vision measurement from the limelight that is being used for pose
   * estimation.
   */
  @Override
  public double getTimeStampSeconds(Limelight limelight) {
    return limelightEstimates[limelight.getId()].timestampSeconds / 1000.0;
  }

  /**
   * Returns the latency in seconds of when the limelight that is being used for pose estimation
   * calculated the robot's pose. It adds the pipeline latency, capture latency, and json parsing
   * latency.
   */
  @Override
  public double getLatencySeconds(Limelight limelight) {
    return (limelightEstimates[limelight.getId()].latency) / 1000.0;
  }

  /** Gets the pose calculated the last time a limelight saw an April Tag */
  @Override
  public Pose2d getLastSeenPose() {
    return lastSeenPose;
  }

  /**
   * Gets the average distance between the specified limelight and the April Tags it sees
   *
   * @param limelightNumber a limelight (SHOOTER, FRONT_LEFT, FRONT_RIGHT).
   * @return the average distance between the robot and the April Tag(s) in meters
   */
  public double getLimelightAprilTagDistance(Limelight limelight) {
    if (canSeeAprilTags(limelight)) {
      return limelightEstimates[limelight.getId()].avgTagDist;
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

  public boolean isLimelightConnected(Limelight limelight) {
    NetworkTable limelightTable = LimelightHelpers.getLimelightNTTable(limelight.getName());
    if (limelightTable.containsKey(limelight.getName())) {
      return true;
    }
    return false;
  }

  /**
   * This checks is there is new pose detected by a limelight, and if so, updates the pose estimate
   *
   * @param limelight the limelight number
   */
  public void checkAndUpdatePose(Limelight limelight) {
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
        double current_TX = LimelightHelpers.getTX(limelight.getName());
        double current_TY = LimelightHelpers.getTY(limelight.getName());

        // This checks if the limelight reading is new. The reasoning being that if the TX and TY
        // are EXACTLY the same, it hasn't updated yet with a new reading. We are doing it this way,
        // because to get the timestamp of the reading, you need to parse the JSON dump which can be
        // very demanding whereas this only has to get the Network Table entries for TX and TY.
        if (current_TX != last_TX || current_TY != last_TY) {
          updatePoseEstimate(limelight);
          limelightThreads.computeIfPresent(
              limelight, (key, value) -> new AtomicReference<>(new VisionInputs()));
          // This is to keep track of the last valid pose calculated by the limelights
          // it is used when the driver resets the robot odometry to the limelight calculated
          // position
          if (canSeeAprilTags(limelight)) {
            lastSeenPose = getMegaTag1PoseEstimate(limelight).pose;
          }
        } else {
          // Retrieve the AtomicBoolean for the given limelight number
          // AtomicReference isThreadRunning =
          //     limelightThreads.getOrDefault(limelight, new AtomicReference<>(new
          // VisionInputs()));
          // // Only stop the thread if it's currently running
          // if (isThreadRunning.get()) {
          // stop the thread for the specified limelight
          stopLimelightThread(limelight);
          // }
        }
        last_TX = current_TX;
        last_TY = current_TY;
      } catch (Exception e) {
        System.err.println(
            "Error communicating with the: " + limelight.getName() + ": " + e.getMessage());
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
   * @param limelight the limelight number
   */
  private void visionThreadTask(Limelight limelight, VisionInputs inputs) {
    try {
      synchronized (inputs) {
        switch (limelight) {
          case SHOOTER -> {
            inputs.isShooterLimelightConnected = isLimelightConnected(Limelight.SHOOTER);
            inputs.shooterMegaTag1Pose =
                MegatagPoseEstimate.fromLimelight(getMegaTag1PoseEstimate(Limelight.SHOOTER));
            inputs.shooterLatency = getLatencySeconds(Limelight.SHOOTER);
            inputs.shooterMegaTag2Pose =
                MegatagPoseEstimate.fromLimelight(getMegaTag2PoseEstimate(Limelight.SHOOTER));
          }
          case FRONT_LEFT -> {
            inputs.isFrontLeftLimelightConnected = isLimelightConnected(Limelight.FRONT_LEFT);
            inputs.frontLeftMegaTag1Pose =
                MegatagPoseEstimate.fromLimelight(getMegaTag1PoseEstimate(Limelight.FRONT_LEFT));
            inputs.frontLeftLatency = getLatencySeconds(Limelight.FRONT_LEFT);
            inputs.frontLeftMegaTag2Pose =
                MegatagPoseEstimate.fromLimelight(getMegaTag2PoseEstimate(Limelight.FRONT_LEFT));
          }
          case FRONT_RIGHT -> {
            inputs.isFrontRightLimelightConnected = isLimelightConnected(Limelight.FRONT_RIGHT);
            inputs.frontRightMegaTag1Pose =
                MegatagPoseEstimate.fromLimelight(getMegaTag1PoseEstimate(Limelight.FRONT_RIGHT));
            inputs.frontRightLatency = getLatencySeconds(Limelight.FRONT_RIGHT);
            inputs.frontRightMegaTag2Pose =
                MegatagPoseEstimate.fromLimelight(getMegaTag2PoseEstimate(Limelight.FRONT_RIGHT));
          }
        }
        limelightThreads.get(limelight).set(inputs);
      }
    } catch (Exception e) {
      System.err.println(
          "Error updating inputs for " + limelight.getName() + ": " + e.getMessage());
    }
  }

  /**
   * Sets the AtomicBoolean 'runningThreads' to false for the specified limelight. Stops the thread
   * for the specified limelight.
   *
   * @param limelight the limelight number
   */
  public void stopLimelightThread(Limelight limelight) {
    threadManager.stopThread(limelight.getName());
  }

  /** Shuts down all the threads. */
  // @Override
  public void endAllThreads() {
    threadManager.shutdownAllThreads();
  }
}
