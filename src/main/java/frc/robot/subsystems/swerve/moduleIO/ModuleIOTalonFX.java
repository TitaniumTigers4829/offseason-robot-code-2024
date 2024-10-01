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
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConfig;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.extras.CANTHINGY;
import frc.robot.extras.CANTHINGY.DeviceCANBus;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;

import java.util.Queue;

public class ModuleIOTalonFX implements ModuleIO {
    // private final String name;
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder turnEncoder;

    private final VoltageOut voltageOut;
    private final DutyCycleOut percentOut;
    private final VelocityVoltage velocityRequest;
    private final MotionMagicVoltage mmPositionRequest;

    private final Queue<Double> driveEncoderUngearedRevolutions;
    private final StatusSignal<Double> driveEncoderUngearedRevolutionsPerSecond, driveMotorAppliedVoltage, driveMotorCurrent;

    private final Queue<Double> steerEncoderAbsolutePositionRevolutions;
    private final StatusSignal<Double> steerEncoderVelocityRevolutionsPerSecond, steerMotorAppliedVolts, steerMotorCurrent;

    private final BaseStatusSignal[] periodicallyRefreshedSignals;

    public ModuleIOTalonFX(ModuleConfig moduleConfig) {
        // this.name = name;
        driveMotor = new TalonFX(moduleConfig.driveMotorChannel(), HardwareConstants.CANIVORE_CAN_BUS_STRING);
        turnMotor = new TalonFX(moduleConfig.turnMotorChannel(), HardwareConstants.CANIVORE_CAN_BUS_STRING);
        turnEncoder = new CANcoder(moduleConfig.turnEncoderChannel(), DeviceCANBus.CANIVORE.name);

        voltageOut = new VoltageOut(0.0);
        percentOut = new DutyCycleOut(0.0);
        velocityRequest = new VelocityVoltage(0.0);
        mmPositionRequest = new MotionMagicVoltage(0.0);

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


        driveEncoderUngearedRevolutions = OdometryThread.registerSignalInput(driveMotor.getPosition());
        driveEncoderUngearedRevolutionsPerSecond = driveMotor.getVelocity();
        driveMotorAppliedVoltage = driveMotor.getMotorVoltage();
        driveMotorCurrent = driveMotor.getSupplyCurrent();

        steerEncoderAbsolutePositionRevolutions = OdometryThread.registerSignalInput(turnEncoder.getAbsolutePosition());
        steerEncoderVelocityRevolutionsPerSecond = turnEncoder.getVelocity();
        steerMotorAppliedVolts = turnMotor.getMotorVoltage();
        steerMotorCurrent = turnMotor.getSupplyCurrent();

        periodicallyRefreshedSignals = new BaseStatusSignal[]{
                driveEncoderUngearedRevolutionsPerSecond,
                driveMotorAppliedVoltage, driveMotorCurrent,
                steerEncoderVelocityRevolutionsPerSecond,
                steerMotorAppliedVolts, steerMotorCurrent
        };

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, periodicallyRefreshedSignals);
        driveMotor.optimizeBusUtilization();
        turnMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.hardwareConnected = BaseStatusSignal.refreshAll(periodicallyRefreshedSignals).isOK();

        inputs.odometryDriveWheelRevolutions = driveEncoderUngearedRevolutions.stream()
                .mapToDouble(value -> value / ModuleConstants.DRIVE_GEAR_RATIO)
                .toArray();
        driveEncoderUngearedRevolutions.clear();
        if (inputs.odometryDriveWheelRevolutions.length > 0)
            inputs.driveWheelFinalRevolutions = inputs.odometryDriveWheelRevolutions[inputs.odometryDriveWheelRevolutions.length-1];

        inputs.odometrySteerPositions = steerEncoderAbsolutePositionRevolutions.stream()
                .map(this::getSteerFacingFromCANCoderReading)
                .toArray(Rotation2d[]::new);
        steerEncoderAbsolutePositionRevolutions.clear();
        if (inputs.odometrySteerPositions.length > 0)
            inputs.steerFacing = inputs.odometrySteerPositions[inputs.odometrySteerPositions.length-1];

        inputs.driveWheelFinalVelocityRevolutionsPerSec = driveEncoderUngearedRevolutionsPerSecond.getValueAsDouble() / ModuleConstants.DRIVE_GEAR_RATIO;
        inputs.driveMotorAppliedVolts = driveMotorAppliedVoltage.getValueAsDouble();
        inputs.driveMotorCurrentAmps = driveMotorCurrent.getValueAsDouble();

        inputs.steerVelocityRadPerSec = Units.rotationsToRadians(steerEncoderVelocityRevolutionsPerSecond.getValueAsDouble());
        inputs.steerMotorAppliedVolts = steerMotorAppliedVolts.getValueAsDouble();
        inputs.steerMotorCurrentAmps = steerMotorCurrent.getValueAsDouble();
    }

    private Rotation2d getSteerFacingFromCANCoderReading(double canCoderReadingRotations) {
        return Rotation2d.fromRotations(canCoderReadingRotations);
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setSteerPowerPercent(double powerPercent) {
       turnMotor.setControl(percentOut.withOutput(powerPercent));
    }

     /**
   * Sets the desired state for the module and sends calculated output from controller to the motor.
   *
   * @param desiredState Desired state with speed and angle.
   */
  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(desiredState, desiredState.angle);

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

    @Override
    public void setSteerBrake(boolean enable) {
        turnMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}
