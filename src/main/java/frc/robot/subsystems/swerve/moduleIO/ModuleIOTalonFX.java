package frc.robot.subsystems.swerve.moduleIO;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.DeviceCANBus;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConfig;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import java.util.Queue;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;
  private final CANcoder turnEncoder;

  private String canBus = "";

  private final VoltageOut voltageOut = new VoltageOut(0.0); // test: .withUpdateFreqHz(0.0);
  private final DutyCycleOut percentOut = new DutyCycleOut(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final MotionMagicVoltage mmPositionRequest = new MotionMagicVoltage(0.0);

  private final Queue<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity, driveMotorAppliedVoltage, driveMotorCurrent;

  private final Queue<Double> turnEncoderAbsolutePosition;
  private final StatusSignal<Double> turnEncoderVelocityPerSecond,
      turnMotorAppliedVolts,
      turnMotorCurrent;

  private final BaseStatusSignal[] periodicallyRefreshedSignals;

  public ModuleIOTalonFX(ModuleConfig moduleConfig) {
    driveMotor = new TalonFX(moduleConfig.driveMotorChannel(), DeviceCANBus.CANIVORE.name);
    turnMotor = new TalonFX(moduleConfig.turnMotorChannel(), DeviceCANBus.CANIVORE.name);
    turnEncoder = new CANcoder(moduleConfig.turnEncoderChannel(), DeviceCANBus.CANIVORE.name);

    CANcoderConfiguration turnEncoderConfig = new CANcoderConfiguration();
    turnEncoderConfig.MagnetSensor.MagnetOffset = -moduleConfig.angleZero();
    turnEncoderConfig.MagnetSensor.SensorDirection = moduleConfig.encoderReversed();
    turnEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turnEncoder.getConfigurator().apply(turnEncoderConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.Slot0.kP = ModuleConstants.DRIVE_P;
    driveConfig.Slot0.kI = ModuleConstants.DRIVE_I;
    driveConfig.Slot0.kD = ModuleConstants.DRIVE_D;
    driveConfig.Slot0.kS = ModuleConstants.DRIVE_S;
    driveConfig.Slot0.kV = ModuleConstants.DRIVE_V;
    driveConfig.Slot0.kA = ModuleConstants.DRIVE_A;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted = moduleConfig.driveReversed();
    driveConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = ModuleConstants.DRIVE_SUPPLY_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimit = ModuleConstants.DRIVE_STATOR_LIMIT;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveMotor.getConfigurator().apply(driveConfig, HardwareConstants.TIMEOUT_S);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.Slot0.kP = ModuleConstants.TURN_P;
    turnConfig.Slot0.kI = ModuleConstants.TURN_I;
    turnConfig.Slot0.kD = ModuleConstants.TURN_D;
    turnConfig.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.MotorOutput.Inverted = moduleConfig.turnReversed();
    turnConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity =
        ModuleConstants.MAX_ANGULAR_SPEED_ROTATIONS_PER_SECOND;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        ModuleConstants.MAX_ANGULAR_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.CurrentLimits.SupplyCurrentLimit = 20;
    turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    turnMotor.getConfigurator().apply(turnConfig, HardwareConstants.TIMEOUT_S);

    drivePosition = OdometryThread.registerSignalInput(driveMotor.getPosition());
    driveVelocity = driveMotor.getVelocity();
    driveMotorAppliedVoltage = driveMotor.getMotorVoltage();
    driveMotorCurrent = driveMotor.getSupplyCurrent();

    turnEncoderAbsolutePosition =
        OdometryThread.registerSignalInput(turnEncoder.getAbsolutePosition());
    turnEncoderVelocityPerSecond = turnEncoder.getVelocity();
    turnMotorAppliedVolts = turnMotor.getMotorVoltage();
    turnMotorCurrent = turnMotor.getSupplyCurrent();

    periodicallyRefreshedSignals =
        new BaseStatusSignal[] {
          driveVelocity,
          driveMotorAppliedVoltage,
          driveMotorCurrent,
          turnEncoderVelocityPerSecond,
          turnMotorAppliedVolts,
          turnMotorCurrent
        };

    driveMotor.setPosition(0.0);
    turnMotor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, periodicallyRefreshedSignals);
    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.isConnected = BaseStatusSignal.refreshAll(periodicallyRefreshedSignals).isOK();

    inputs.odometryDriveWheelRevolutions =
        drivePosition.stream()
            .mapToDouble(value -> value / ModuleConstants.DRIVE_GEAR_RATIO)
            .toArray();
    drivePosition.clear();
    if (inputs.odometryDriveWheelRevolutions.length > 0)
      inputs.drivePosition =
          inputs.odometryDriveWheelRevolutions[inputs.odometryDriveWheelRevolutions.length - 1];

    inputs.odometryTurnPositions =
        turnEncoderAbsolutePosition.stream()
            .map(this::getTurnAbsolutePosition)
            .toArray(Rotation2d[]::new);
    turnEncoderAbsolutePosition.clear();
    if (inputs.odometryTurnPositions.length > 0)
      inputs.turnRotation = inputs.odometryTurnPositions[inputs.odometryTurnPositions.length - 1];

    inputs.driveWheelFinalVelocityPerSec =
        driveVelocity.getValueAsDouble() / ModuleConstants.DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveMotorAppliedVoltage.getValueAsDouble();
    inputs.driveCurrentAmps = driveMotorCurrent.getValueAsDouble();

    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnEncoderVelocityPerSecond.getValueAsDouble());
    inputs.turnMotorAppliedVolts = turnMotorAppliedVolts.getValueAsDouble();
    inputs.turnMotorCurrentAmps = turnMotorCurrent.getValueAsDouble();
  }

  private Rotation2d getTurnAbsolutePosition(double canCoderReadingRotations) {
    return Rotation2d.fromRotations(canCoderReadingRotations);
  }

  @Override
  public void setDriveSpeed(double volts) {
    driveMotor.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setTurnSpeed(double powerPercent) {
    turnMotor.setControl(percentOut.withOutput(powerPercent));
  }

  /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   *
   * @param desiredState Desired state with speed and angle.
   */
  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    double turnRotations = getTurnRotations();
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(
            desiredState, new Rotation2d(Units.rotationsToRadians(turnRotations)));

    if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.01) {
      driveMotor.set(0);
      turnMotor.set(0);
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

  @Override
  public void setDriveBrake(boolean enable) {
    driveMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  public double getTurnRotations() {
    turnEncoder.getAbsolutePosition().refresh();
    return turnEncoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void setTurnBrake(boolean enable) {
    turnMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void stopModule() {
    driveMotor.stopMotor();
    turnMotor.stopMotor();
  }
}
