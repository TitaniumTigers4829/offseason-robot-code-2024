package frc.robot.extras.simulation.mechanismSim.swerve;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.extras.simulation.field.SimulatedField.SIMULATION_DT;
import static frc.robot.extras.simulation.field.SimulatedField.SIMULATION_SUB_TICKS_IN_1_PERIOD;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Supplier;
import org.dyn4j.geometry.Vector2;

public class SwerveModuleSimulation {
  private final DCMotor DRIVE_MOTOR;
  private final BrushlessMotorSim turnMotorSim;
  private final double DRIVE_CURRENT_LIMIT,
      DRIVE_GEAR_RATIO,
      TURN_GEAR_RATIO,
      DRIVE_FRICTION_VOLTAGE,
      TURN_FRICTION_VOLTAGE,
      WHEELS_COEFFICIENT_OF_FRICTION,
      WHEEL_RADIUS_METERS,
      DRIVE_WHEEL_INERTIA = 0.01;
  private double driveMotorRequestedVolts = 0.0,
      turnMotorAppliedVolts = 0.0,
      driveMotorAppliedVolts = 0.0,
      driveMotorSupplyCurrentAmps = 0.0,
      turnMotorSupplyCurrentAmps = 0.0,
      turnRelativeEncoderPositionRad = 0.0,
      turnRelativeEncoderSpeedRadPerSec = 0.0,
      turnAbsoluteEncoderSpeedRadPerSec = 0.0,
      driveEncoderUnGearedPositionRad = 0.0,
      driveEncoderUnGearedSpeedRadPerSec = 0.0;
  private Rotation2d turnAbsoluteRotation = Rotation2d.fromRotations(Math.random());

  private final double turnRelativeEncoderOffSet = (Math.random() - 0.5) * 30;

  private final Queue<Double> cachedTurnRelativeEncoderPositionsRad,
      cachedDriveEncoderUnGearedPositionsRad;
  private final Queue<Rotation2d> cachedTurnAbsolutePositions;

  public SwerveModuleSimulation(
      DCMotor driveMotor,
      DCMotor turnMotor,
      double driveCurrentLimit,
      double driveGearRatio,
      double turnGearRatio,
      double driveFrictionVoltage,
      double turnFrictionVoltage,
      double tireCoefficientOfFriction,
      double wheelsRadiusMeters,
      double turnRotationalInertia) {
    DRIVE_MOTOR = driveMotor;
    DRIVE_CURRENT_LIMIT = driveCurrentLimit;
    DRIVE_GEAR_RATIO = driveGearRatio;
    TURN_GEAR_RATIO = turnGearRatio;
    DRIVE_FRICTION_VOLTAGE = driveFrictionVoltage;
    TURN_FRICTION_VOLTAGE = turnFrictionVoltage;
    WHEELS_COEFFICIENT_OF_FRICTION = tireCoefficientOfFriction;
    WHEEL_RADIUS_METERS = wheelsRadiusMeters;

    this.turnMotorSim =
        new BrushlessMotorSim(
            turnMotor, TURN_GEAR_RATIO, turnRotationalInertia, TURN_FRICTION_VOLTAGE);

    this.cachedDriveEncoderUnGearedPositionsRad = new ConcurrentLinkedQueue<>();
    for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++)
      cachedDriveEncoderUnGearedPositionsRad.offer(driveEncoderUnGearedPositionRad);
    this.cachedTurnRelativeEncoderPositionsRad = new ConcurrentLinkedQueue<>();
    for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++)
      cachedTurnRelativeEncoderPositionsRad.offer(turnRelativeEncoderPositionRad);
    this.cachedTurnAbsolutePositions = new ConcurrentLinkedQueue<>();
    for (int i = 0; i < SIMULATION_SUB_TICKS_IN_1_PERIOD; i++)
      cachedTurnAbsolutePositions.offer(turnAbsoluteRotation);

    this.turnRelativeEncoderPositionRad =
        turnAbsoluteRotation.getRadians() + turnRelativeEncoderOffSet;
  }

  public void requestDriveVoltageOut(double volts) {
    this.driveMotorRequestedVolts = volts;
  }

  public void requestTurnVoltageOut(double volts) {
    this.turnMotorAppliedVolts = volts;
    // this.turnMotorSim.setInputVoltage(MathUtil.applyDeadband(volts, turn_FRICTION_VOLTAGE,
    // 12));
  }

  public void requestDriveVoltageOut(Voltage volts) {
    this.driveMotorRequestedVolts = volts.in(Volts);
  }

  public void requestTurnVoltageOut(Voltage volts) {
    this.turnMotorAppliedVolts = volts.in(Volts);
    // this.turnMotorSim.setInputVoltage(MathUtil.applyDeadband(volts, turn_FRICTION_VOLTAGE,
    // 12));
  }

  public double getDriveMotorAppliedVolts() {
    return driveMotorAppliedVolts;
  }

  public double getTurnMotorAppliedVolts() {
    return turnMotorAppliedVolts;
  }

  public double getDriveMotorSupplyCurrentAmps() {
    return driveMotorSupplyCurrentAmps;
  }

  public double getTurnMotorSupplyCurrentAmps() {
    return turnMotorSupplyCurrentAmps;
  }

  public double getDriveEncoderUnGearedPositionRad() {
    return driveEncoderUnGearedPositionRad;
  }

  public double getDriveEncoderFinalPositionRad() {
    return getDriveEncoderUnGearedPositionRad() / DRIVE_GEAR_RATIO;
  }

  public double getDriveEncoderUnGearedSpeedRadPerSec() {
    return driveEncoderUnGearedSpeedRadPerSec;
  }

  public double getDriveWheelFinalSpeedRadPerSec() {
    return getDriveEncoderUnGearedSpeedRadPerSec() / DRIVE_GEAR_RATIO;
  }

  /** geared */
  public double getTurnRelativeEncoderPositionRad() {
    return turnRelativeEncoderPositionRad;
  }

  /** geared */
  public double getTurnRelativeEncoderSpeedRadPerSec() {
    return turnRelativeEncoderSpeedRadPerSec;
  }

  public Rotation2d getTurnAbsolutePosition() {
    return turnAbsoluteRotation;
  }

  public double getTurnAbsoluteEncoderSpeedRadPerSec() {
    return turnAbsoluteEncoderSpeedRadPerSec;
  }

  public double[] getCachedDriveEncoderUnGearedPositionsRad() {
    return cachedDriveEncoderUnGearedPositionsRad.stream().mapToDouble(value -> value).toArray();
  }

  public double[] getCachedDriveWheelFinalPositionsRad() {
    return cachedDriveEncoderUnGearedPositionsRad.stream()
        .mapToDouble(value -> value / DRIVE_GEAR_RATIO)
        .toArray();
  }

  public double[] getCachedTurnRelativeEncoderPositions() {
    return cachedTurnRelativeEncoderPositionsRad.stream().mapToDouble(value -> value).toArray();
  }

  public Rotation2d[] getCachedTurnAbsolutePositions() {
    return cachedTurnAbsolutePositions.toArray(Rotation2d[]::new);
  }

  protected double getGrippingForceNewtons(double gravityForceOnModuleNewtons) {
    return gravityForceOnModuleNewtons * WHEELS_COEFFICIENT_OF_FRICTION;
  }

  /**
   * updates the simulation sub-tick for this module, updating its inner status (sensor readings)
   * and calculating a total force
   *
   * @param moduleCurrentGroundVelocityWorldRelative
   * @return the propelling force that the module generates
   */
  public Vector2 updateSimulationSubTickGetModuleForce(
      Vector2 moduleCurrentGroundVelocityWorldRelative,
      Rotation2d robotRotation,
      double gravityForceOnModuleNewtons) {
    updateturnSimulation();

    /* the maximum gripping force that the wheel can generate */
    final double grippingForceNewtons = getGrippingForceNewtons(gravityForceOnModuleNewtons);
    final Rotation2d moduleWorldRotation = this.turnAbsoluteRotation.plus(robotRotation);
    final Vector2 propellingForce =
        getPropellingForce(
            grippingForceNewtons, moduleWorldRotation, moduleCurrentGroundVelocityWorldRelative);
    updateEncoderTicks();

    return propellingForce;
  }

  /** */
  private void updateturnSimulation() {
    turnMotorSim.update(SIMULATION_DT);

    /* update the readings of the sensor */
    this.turnAbsoluteRotation = Rotation2d.fromRadians(turnMotorSim.getAngularPositionRad());
    this.turnRelativeEncoderPositionRad =
        turnMotorSim.getAngularPositionRad() + turnRelativeEncoderOffSet;
    this.turnAbsoluteEncoderSpeedRadPerSec = turnMotorSim.getAngularVelocityRadPerSec();
    this.turnRelativeEncoderSpeedRadPerSec = turnAbsoluteEncoderSpeedRadPerSec * TURN_GEAR_RATIO;

    /* cache sensor readings to queue for high-frequency odometry */
    this.cachedTurnAbsolutePositions.poll();
    this.cachedTurnAbsolutePositions.offer(turnAbsoluteRotation);
    this.cachedTurnRelativeEncoderPositionsRad.poll();
    this.cachedTurnRelativeEncoderPositionsRad.offer(turnRelativeEncoderPositionRad);
  }

  private Vector2 getPropellingForce(
      double grippingForceNewtons,
      Rotation2d moduleWorldRotation,
      Vector2 moduleCurrentGroundVelocity) {
    final double driveWheelTorque = getDriveWheelTorque(),
        theoreticalMaxPropellingForceNewtons = driveWheelTorque / WHEEL_RADIUS_METERS;
    final boolean skidding = Math.abs(theoreticalMaxPropellingForceNewtons) > grippingForceNewtons;
    final double propellingForceNewtons;
    if (skidding)
      propellingForceNewtons =
          Math.copySign(grippingForceNewtons, theoreticalMaxPropellingForceNewtons);
    else propellingForceNewtons = theoreticalMaxPropellingForceNewtons;

    final double floorVelocityProjectionOnWheelDirectionMPS =
        moduleCurrentGroundVelocity.getMagnitude()
            * Math.cos(
                moduleCurrentGroundVelocity.getAngleBetween(
                    new Vector2(moduleWorldRotation.getRadians())));

    if (skidding) {
      /* if the chassis is skidding, part of the toque will cause the wheels to spin freely */
      final double torqueOnWheel = driveWheelTorque * 0.3;
      this.driveEncoderUnGearedSpeedRadPerSec +=
          torqueOnWheel / DRIVE_WHEEL_INERTIA * SIMULATION_DT * DRIVE_GEAR_RATIO;
    } else // if the chassis is tightly gripped on floor, the floor velocity is projected to the
      // wheel
      this.driveEncoderUnGearedSpeedRadPerSec =
          floorVelocityProjectionOnWheelDirectionMPS / WHEEL_RADIUS_METERS * DRIVE_GEAR_RATIO;

    return Vector2.create(propellingForceNewtons, moduleWorldRotation.getRadians());
  }

  private double getDriveWheelTorque() {
    final double currentAtRequestedVolts =
        DRIVE_MOTOR.getCurrent(this.driveEncoderUnGearedSpeedRadPerSec, driveMotorRequestedVolts);

    driveMotorAppliedVolts = driveMotorRequestedVolts;
    /* normally, motor controller starts cutting the supply voltage when the current exceed 150% the current limit */
    final boolean currentTooHigh = Math.abs(currentAtRequestedVolts) > 1.2 * DRIVE_CURRENT_LIMIT,
        driveMotorTryingToAccelerate = driveMotorRequestedVolts * driveMotorSupplyCurrentAmps > 0;

    if (currentTooHigh && driveMotorTryingToAccelerate) {
      /* activate current limit, cut down the applied voltage to match current limit */
      final double currentWithLimits = Math.copySign(DRIVE_CURRENT_LIMIT, currentAtRequestedVolts);
      driveMotorAppliedVolts =
          DRIVE_MOTOR.getVoltage(
              DRIVE_MOTOR.getTorque(currentWithLimits), this.driveEncoderUnGearedSpeedRadPerSec);
    }

    driveMotorAppliedVolts = MathUtil.clamp(driveMotorAppliedVolts, -12, 12);

    /* calculate the actual supply current */
    driveMotorSupplyCurrentAmps =
        DRIVE_MOTOR.getCurrent(
            this.driveEncoderUnGearedSpeedRadPerSec,
            MathUtil.applyDeadband(driveMotorAppliedVolts, DRIVE_FRICTION_VOLTAGE, 12));

    /* calculate the torque generated,  */
    final double torqueOnRotor = DRIVE_MOTOR.getTorque(driveMotorSupplyCurrentAmps);
    return torqueOnRotor * DRIVE_GEAR_RATIO;
  }

  /**
   * @return the current module state of this simulation module
   */
  protected SwerveModuleState getCurrentState() {
    return new SwerveModuleState(
        getDriveWheelFinalSpeedRadPerSec() * WHEEL_RADIUS_METERS, turnAbsoluteRotation);
  }

  /**
   * gets the state of the module, if it is allowed to spin freely for a long time under the current
   * applied drive volts
   *
   * @return the free spinning module state
   */
  protected SwerveModuleState getFreeSpinState() {
    return new SwerveModuleState(
        DRIVE_MOTOR.getSpeed(
                DRIVE_MOTOR.getTorque(DRIVE_MOTOR.getCurrent(0, DRIVE_FRICTION_VOLTAGE)),
                driveMotorAppliedVolts)
            / DRIVE_GEAR_RATIO
            * WHEEL_RADIUS_METERS,
        turnAbsoluteRotation);
  }

  private void updateEncoderTicks() {
    this.driveEncoderUnGearedPositionRad += this.driveEncoderUnGearedSpeedRadPerSec * SIMULATION_DT;
    this.cachedDriveEncoderUnGearedPositionsRad.poll();
    this.cachedDriveEncoderUnGearedPositionsRad.offer(driveEncoderUnGearedPositionRad);
  }

  public double getModuleTheoreticalSpeedMPS() {
    return DRIVE_MOTOR.freeSpeedRadPerSec / DRIVE_GEAR_RATIO * WHEEL_RADIUS_METERS;
  }

  public double getTheoreticalPropellingForcePerModule(double robotMass, int modulesCount) {
    final double
        maxThrustNewtons =
            DRIVE_MOTOR.getTorque(DRIVE_CURRENT_LIMIT) * DRIVE_GEAR_RATIO / WHEEL_RADIUS_METERS,
        maxGrippingNewtons = 9.8 * robotMass / modulesCount * WHEELS_COEFFICIENT_OF_FRICTION;

    return Math.min(maxThrustNewtons, maxGrippingNewtons);
  }

  public double getModuleMaxAccelerationMPSsq(double robotMass, int modulesCount) {
    return getTheoreticalPropellingForcePerModule(robotMass, modulesCount)
        * modulesCount
        / robotMass;
  }

  public enum DRIVE_WHEEL_TYPE {
    RUBBER,
    TIRE
  }

  /**
   * creates a <a href="https://www.swervedrivespecialties.com/products/mk4n-swerve-module">SDS
   * Mark4-n Swerve Module</a> for simulation
   */
  public static Supplier<SwerveModuleSimulation> getModule(
      DCMotor driveMotor,
      DCMotor turnMotor,
      double driveCurrentLimitAmps,
      DRIVE_WHEEL_TYPE driveWheelType,
      double driveGearRatio) {
    return () ->
        new SwerveModuleSimulation(
            driveMotor,
            turnMotor,
            driveCurrentLimitAmps,
            driveGearRatio,
            11.3142,
            0.25,
            0.05,
            switch (driveWheelType) {
              case RUBBER -> 1.55;
              case TIRE -> 1.45;
            },
            Units.inchesToMeters(2),
            0.05);
  }
}
