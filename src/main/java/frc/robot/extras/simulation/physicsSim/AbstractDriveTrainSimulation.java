package frc.robot.extras.simulation.physicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.extras.util.GeomUtil;
import org.dyn4j.dynamics.Body;
import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;

/**
 *
 *
 * <h1>Represents an Abstract Drivetrain Simulation.</h1>
 *
 * <h3>Simulates the Mass, Collision Space, and Friction of the Drivetrain.</h3>
 *
 * <p>This class models the physical properties of a drivetrain, including mass, collision space,
 * and friction.
 *
 * <p>The propelling forces generated by motors are simulated in its subclasses, such as {@link
 * SimplifiedHolonomicDriveSimulation} or {@link SwerveDriveSimulation}.
 */
public abstract class AbstractDriveTrainSimulation extends Body {
  public static final double
      BUMPER_COEFFICIENT_OF_FRICTION =
          0.65, // https://en.wikipedia.org/wiki/Friction#Coefficient_of_friction
      BUMPER_COEFFICIENT_OF_RESTITUTION =
          0.08; // https://simple.wikipedia.org/wiki/Coefficient_of_restitution

  /**
   *
   *
   * <h2>Properties of the drive train simulation</h2>
   */
  public final DriveTrainSimulationProfile profile;

  /**
   *
   *
   * <h2>Creates a Simulation of a Drivetrain.</h2>
   *
   * <h3>Sets Up the Collision Space and Mass of the Chassis.</h3>
   *
   * <p>Since this is an abstract class, the constructor must be called from a subclass.
   *
   * <p>Note that the chassis does not appear on the simulation field upon creation. Refer to {@link
   * org.ironmaple.simulation.SimulatedArena} for instructions on how to add it to the simulation
   * world.
   *
   * @param profile a profile class that stores all the configurations for this drivetrain, see
   *     {@link DriveTrainSimulationProfile}
   * @param initialPoseOnField the initial pose of the drivetrain in the simulation world
   */
  protected AbstractDriveTrainSimulation(
      DriveTrainSimulationProfile profile, Pose2d initialPoseOnField) {
    this.profile = profile;

    /* width and height in world reference is flipped */
    final double WIDTH_IN_WORLD_REFERENCE = profile.length,
        HEIGHT_IN_WORLD_REFERENCE = profile.width;

    super.addFixture(
        Geometry.createRectangle(WIDTH_IN_WORLD_REFERENCE, HEIGHT_IN_WORLD_REFERENCE),
        profile.robotMass / (profile.length * profile.width),
        BUMPER_COEFFICIENT_OF_FRICTION,
        BUMPER_COEFFICIENT_OF_RESTITUTION);

    super.setMass(MassType.NORMAL);
    super.setLinearDamping(profile.linearVelocityDamping);
    super.setAngularDamping(profile.angularVelocityDamping);
    setSimulationWorldPose(initialPoseOnField);
  }

  /**
   *
   *
   * <h2>Sets the Robot's Current Pose in the Simulation World.</h2>
   *
   * <p>This method instantly teleports the robot to the specified pose in the simulation world. The
   * robot does not drive to the new pose; it is moved directly.
   *
   * @param robotPose the desired robot pose, represented as a {@link Pose2d}
   */
  public void setSimulationWorldPose(Pose2d robotPose) {
    super.transform.set(GeomUtil.toDyn4jTransform(robotPose));
    super.linearVelocity.set(0, 0);
  }

  /**
   *
   *
   * <h2>Sets the Robot's Speeds to the Given Chassis Speeds.</h2>
   *
   * <p>This method sets the robot's current velocity to the specified chassis speeds.
   *
   * <p>The robot does not accelerate smoothly to these speeds; instead, it jumps to the velocity
   * <strong>Instantaneously</strong>.
   *
   * @param givenSpeeds the desired chassis speeds, represented as a {@link ChassisSpeeds} object
   */
  public void setRobotSpeeds(ChassisSpeeds givenSpeeds) {
    super.setLinearVelocity(GeomUtil.toDyn4jLinearVelocity(givenSpeeds));
    super.setAngularVelocity(givenSpeeds.omegaRadiansPerSecond);
  }

  /**
   *
   *
   * <h2>Abstract Simulation Sub-Tick Method.</h2>
   *
   * <p>This method is called every time the simulation world is updated.
   *
   * <p>It is implemented in the sub-classes of {@link AbstractDriveTrainSimulation}.
   *
   * <p>It is responsible for applying the propelling forces to the robot during each sub-tick of
   * the simulation.
   */
  public abstract void simulationSubTick();

  /**
   *
   *
   * <h2>Simulates the Linear Friction Force on the Drivetrain.</h2>
   *
   * <p>This method simulates the linear friction forces on the drivetrain and applies them to the
   * physics engine.
   *
   * <p>It should be called from the implementation of {@link #simulationSubTick()}, particularly
   * when the drivetrain is not attempting to move linearly.
   */
  protected void simulateChassisLinearFriction() {
    final double actualLinearVelocityPercent =
        getLinearVelocity().getMagnitude() / profile.maxLinearVelocity;
    final boolean robotActuallyMovingLinearly = actualLinearVelocityPercent > 0.01;
    if (robotActuallyMovingLinearly)
      /*
       * apply the friction force
       * with its direction opposite to the that of the current velocity
       * with its magnitude specified in profile
       * */
      super.applyForce(
          new Force(
              super.linearVelocity
                  .getNormalized()
                  .multiply(-profile.frictionForceMagnitudeNewtons)));
    else /* when the velocity is too small, just set the object at rest */
      super.setLinearVelocity(new Vector2());
  }

  /**
   *
   *
   * <h2>Simulates the Angular Friction Force on the Drivetrain.</h2>
   *
   * <p>This method simulates the angular friction forces on the drivetrain and applies them to the
   * physics engine.
   *
   * <p>It should be called from the implementation of {@link #simulationSubTick()}, particularly
   * when the drivetrain is not attempting to move rotationally.
   */
  protected void simulateChassisAngularFriction() {
    final double actualRotationalMotionPercent =
        Math.abs(getAngularVelocity() / profile.maxAngularVelocity);
    if (actualRotationalMotionPercent > 0.01)
      /*
       * apply the friction torque
       * with its sign opposite to the that of the current angular velocity
       * with its magnitude specified in profile
       * */
      super.applyTorque(
          Math.copySign(this.profile.angularFrictionTorqueMagnitude, -super.getAngularVelocity()));
    else
      /* when the velocity is too small, just set it still */
      super.setAngularVelocity(0);
  }

  /**
   *
   *
   * <h2>Gets the Actual Pose of the Drivetrain in the Simulation World.</h2>
   *
   * <p>This method is used to display the robot on <a
   * href="https://docs.advantagescope.org/tab-reference/3d-field/">AdvantageScope Field3d</a> or to
   * <a
   * href="https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html#updating-the-simulation-world">update
   * vision simulations</a>.
   *
   * <p><strong>Note:</strong> Do not use this method to simulate odometry! For a more realistic
   * odometry simulation, use a {@link SwerveDriveSimulation} together with a {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}.
   *
   * @return a {@link Pose2d} object yielding the current world pose of the robot in the simulation
   */
  public Pose2d getSimulatedDriveTrainPose() {
    return GeomUtil.toWpilibPose2d(getTransform());
  }

  /**
   *
   *
   * <h2>Gets the Actual Robot-Relative Chassis Speeds from the Simulation.</h2>
   *
   * <p>This method returns the actual chassis speeds of the drivetrain in the simulation, relative
   * to the robot.
   *
   * <p>To simulate the chassis speeds calculated by encoders, use a {@link SwerveDriveSimulation}
   * together with {@link
   * edu.wpi.first.math.kinematics.SwerveDriveKinematics#toChassisSpeeds(SwerveModuleState...)} for
   * a more realistic simulation.
   *
   * @return the actual chassis speeds in the simulation world, <strong>Robot-Relative</strong>
   */
  public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsRobotRelative() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        getSimulatedDriveTrainPose().getRotation());
  }

  /**
   *
   *
   * <h2>Gets the Actual Field-Relative Chassis Speeds from the Simulation.</h2>
   *
   * <p>This method returns the actual chassis speeds of the drivetrain in the simulation, relative
   * to the robot.
   *
   * <p>To simulate the chassis speeds calculated by encoders, use a {@link SwerveDriveSimulation}
   * together with {@link
   * edu.wpi.first.math.kinematics.SwerveDriveKinematics#toChassisSpeeds(SwerveModuleState...)} for
   * a more realistic simulation.
   *
   * @return the actual chassis speeds in the simulation world, <strong>Field-Relative</strong>
   */
  public ChassisSpeeds getDriveTrainSimulatedChassisSpeedsFieldRelative() {
    return GeomUtil.toWpilibChassisSpeeds(getLinearVelocity(), getAngularVelocity());
  }

  /** stores the profile of a drivetrain simulation */
  public static final class DriveTrainSimulationProfile {
    public final double maxLinearVelocity,
        maxLinearAcceleration,
        maxAngularVelocity,
        maxAngularAcceleration,
        robotMass,
        width,
        length;
    private double frictionForceMagnitudeNewtons,
        linearVelocityDamping,
        angularFrictionTorqueMagnitude,
        angularVelocityDamping;

    public DriveTrainSimulationProfile(
        double maxLinearVelocity,
        double maxLinearAcceleration,
        double maxAngularVelocity,
        double maxAngularAcceleration,
        double robotMass,
        double width,
        double length) {
      this.maxLinearVelocity = maxLinearVelocity;
      this.maxLinearAcceleration = maxLinearAcceleration;
      this.maxAngularVelocity = maxAngularVelocity;
      this.maxAngularAcceleration = maxAngularAcceleration;
      this.robotMass = robotMass;
      this.width = width;
      this.length = length;

      final double GRAVITY_CONSTANT = 9.8,
          WHEEL_COEFFICIENT_OF_FRICTION = 0.8,
          DRIVE_BASE_RADIUS = Math.hypot(width / 2, length / 2);
      this.frictionForceMagnitudeNewtons =
          GRAVITY_CONSTANT * WHEEL_COEFFICIENT_OF_FRICTION * robotMass;
      this.linearVelocityDamping = maxLinearAcceleration / maxLinearVelocity * 0.75;
      this.angularFrictionTorqueMagnitude = frictionForceMagnitudeNewtons * DRIVE_BASE_RADIUS / 2;
      this.angularVelocityDamping = maxAngularAcceleration / maxAngularVelocity * 0.75;
    }

    public DriveTrainSimulationProfile withFrictionForceMagnitude(
        double frictionForceMagnitudeNewtons) {
      this.frictionForceMagnitudeNewtons = frictionForceMagnitudeNewtons;
      return this;
    }

    public DriveTrainSimulationProfile withLinearVelocityDamping(double linearVelocityDamping) {
      this.linearVelocityDamping = linearVelocityDamping;
      return this;
    }

    public DriveTrainSimulationProfile withAngularFrictionTorqueMagnitude(
        double angularFrictionTorqueMagnitude) {
      this.angularFrictionTorqueMagnitude = angularFrictionTorqueMagnitude;
      return this;
    }

    public DriveTrainSimulationProfile withAngularVelocityDamping(double angularVelocityDamping) {
      this.angularVelocityDamping = angularVelocityDamping;
      return this;
    }

    @Override
    public String toString() {
      return String.format(
          "RobotProfile { robotMaxVelocity=%.2f(m/s), robotMaxAcceleration=%.2f(m/s^2), robotMass=%.2f(kg), "
              + "frictionForceMagnitude=%.2f(N), linearVelocityDamping=%.2f(N*s*m^-1), maxAngularVelocity=%.2f(rad/s), "
              + "maxAngularAcceleration=%.2f (rad/s^2), angularDamping=%.2f(N*m*s*rad^-1), angularFrictionTorqueMagnitude=%.2f(N*m), width=%.2f(m), "
              + "length=%.2f(m) }",
          maxLinearVelocity,
          maxLinearAcceleration,
          robotMass,
          frictionForceMagnitudeNewtons,
          linearVelocityDamping,
          maxAngularVelocity,
          maxAngularAcceleration,
          angularVelocityDamping,
          angularFrictionTorqueMagnitude,
          width,
          length);
    }
  }
}
