package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.extras.purplelib.LoggedCANCoder;
import frc.robot.extras.purplelib.LoggedTalonFX;
import frc.robot.extras.purplelib.LoggedTalonFX.FeedbackSensor;

public class SwerveModule {

  private final LoggedCANCoder turnEncoder;
  private final LoggedTalonFX driveMotor;
  private final LoggedTalonFX turnMotor;

  private final StatusSignal<Double> driveMotorVelocity;
  private final StatusSignal<Double> driveMotorPosition;
  private final StatusSignal<Double> turnEncoderPos;

  private final MotionMagicVoltage mmPositionRequest;
  private final VelocityVoltage velocityRequest;

  private String name;

  /**
   * Constructs a swerve module
   *
   * @param driveMotorChannel ID of the drive motor
   * @param turnMotorChannel ID of the turn motor
   * @param turnEncoderChannel ID of the CANCoder
   * @param angleZero CANCoder offset
   * @param encoderReversed is the turn encoder reversed
   * @param turnReversed is the turn motor reversed
   * @param driveReversed is the drive motor reversed
   * @param name name of the motor
   */
  public SwerveModule(
      int driveMotorChannel,
      int turnMotorChannel,
      int turnEncoderChannel,
      double angleZero,
      SensorDirectionValue encoderReversed,
      InvertedValue turnReversed,
      InvertedValue driveReversed,
      String name) {
    this.name = name;

    turnEncoder = new LoggedCANCoder(turnEncoderChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING, name);
    driveMotor = new LoggedTalonFX(driveMotorChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING, name);
    turnMotor = new LoggedTalonFX(turnMotorChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING, name);

    mmPositionRequest = new MotionMagicVoltage(0);
    velocityRequest = new VelocityVoltage(0);

    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.MagnetOffset = -angleZero;
    turnEncoderConfig.MagnetSensor.SensorDirection = encoderReversed;
    turnEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turnEncoder.applyConfigs(turnEncoderConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Slot0.kP = ModuleConstants.DRIVE_P;
    driveConfig.Slot0.kI = ModuleConstants.DRIVE_I;
    driveConfig.Slot0.kD = ModuleConstants.DRIVE_D;
    driveConfig.Slot0.kS = ModuleConstants.DRIVE_S;
    driveConfig.Slot0.kV = ModuleConstants.DRIVE_V;
    driveConfig.Slot0.kA = ModuleConstants.DRIVE_A;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = driveReversed;
    driveConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVE_SUPPLY_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimit = ModuleConstants.DRIVE_STATOR_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveMotor.applyConfigs(driveConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();

    turnMotor.initializeFeedbackSensor(turnEncoder, FeedbackSensor.REMOTE);
    turnMotor.initializeTalonPID(ModuleConstants.TURN_P, ModuleConstants.TURN_I, ModuleConstants.TURN_D);
    turnMotor.initializeMotionMagic(ModuleConstants.MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND, ModuleConstants.MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED);
    turnMotor.setNeutralMode(NeutralModeValue.Brake);
    turnMotor.setInvert(turnReversed);
    turnConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

    turnMotor.initializeSupplyCurrentLimits(20, true, HardwareConstants.TIMEOUT_S);
    turnMotor.applyConfigs(turnConfig, HardwareConstants.TIMEOUT_S);

    turnEncoderPos = turnEncoder.getAbsolutePosition();
    driveMotorPosition = driveMotor.getSelectedSensorPosition();
    driveMotorVelocity = driveMotor.getSelectedSensorVelocity();

    driveMotor.setPosition(0);
    turnMotor.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.SIGNAL_FREQUENCY, turnEncoderPos, driveMotorPosition, driveMotorVelocity);

    driveMotor.optimizeBusUtilization(HardwareConstants.SIGNAL_FREQUENCY);
    turnMotor.optimizeBusUtilization(HardwareConstants.SIGNAL_FREQUENCY);

    // ParentDevice.optimizeBusUtilizationForAll(turnEncoder, driveMotor, turnMotor);
  }

  /**
   * Gets the heading of the module
   *
   * @return the absolute position of the CANCoder
   */
  public double getModuleHeading() {
    turnEncoderPos.refresh();
    return turnEncoderPos.getValue();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    driveMotorVelocity.refresh();

    double speedMetersPerSecond =
        ModuleConstants.DRIVE_TO_METERS_PER_SECOND * driveMotorVelocity.getValueAsDouble();

    return new SwerveModuleState(
        speedMetersPerSecond, Rotation2d.fromRotations(getModuleHeading()));
  }

  /**
   * Gets the module position consisting of the distance it has traveled and the angle it is
   * rotated.
   *
   * @return a SwerveModulePosition object containing position and rotation
   */
  public SwerveModulePosition getPosition() {
    driveMotorPosition.refresh();
    double position = ModuleConstants.DRIVE_TO_METERS * -driveMotorPosition.getValue();
    Rotation2d rotation = Rotation2d.fromRotations(getModuleHeading());
    return new SwerveModulePosition(position, rotation);
  }

  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    double turnRadians = getTurnRadians();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(desiredState, new Rotation2d(turnRadians));

    if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.01) {
      driveMotor.setControl(new DutyCycleOut(0));
      turnMotor.setControl(new DutyCycleOut(0));
      return;
    }

    // Converts meters per second to rotations per second
    double desiredDriveRPS =
        optimizedDesiredState.speedMetersPerSecond
            * ModuleConstants.DRIVE_GEAR_RATIO
            / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;

    driveMotor.setControl(velocityRequest.withVelocity(desiredDriveRPS));
    turnMotor.setControl(
        mmPositionRequest.withPosition(optimizedDesiredState.angle.getRotations()));
  }

  /**
   * Gets the current position of the CANCoder in relation to the magnet
   *
   * @return current CANCoder position in radians
   */
  public double getTurnRadians() {
    turnEncoderPos.refresh();
    return Rotation2d.fromRotations(turnEncoderPos.getValue()).getRadians();
  }

  /**
   * Gets the drive position using the internal encoder
   *
   * @return the drive position in radians
   */
  public double getDrivePositionRadians() {
    driveMotorPosition.refresh();
    return 2.0
        * Math.PI
        * (driveMotorPosition.getValue() / Constants.ModuleConstants.DRIVE_GEAR_RATIO);
  }

  /** This is called in the periodic of DriveSubsystem */
  public void periodicFunction() {}
}