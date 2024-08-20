// package frc.robot.subsystems.swerve;

// import com.ctre.phoenix6.BaseStatusSignal;
// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.NeutralOut;
// import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.util.Units;
// import frc.robot.Constants.HardwareConstants;
// import frc.robot.Constants.ModuleConstants;

// import java.util.Queue;

// /**
//  * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
//  * CANcoder
//  *
//  * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
//  * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
//  *
//  * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
//  * motion on the drive motor will propel the robot forward) and copy the reported values from the
//  * absolute encoders using AdvantageScope. These values are logged under
//  * "/Drive/ModuleX/TurnAbsolutePositionRad"
//  */
// public class ModuleIOTalonFX implements ModuleIO {
//   private final TalonFX driveTalon;
//   private final TalonFX turnTalon;
//   private final CANcoder cancoder;

//   private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
//   private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();

//   // private final Queue<Double> timestampQueue;

//   private final StatusSignal<Double> drivePosition;
//   private final Queue<Double> drivePositionQueue;
//   private final StatusSignal<Double> driveVelocity;
//   private final StatusSignal<Double> driveAppliedVolts;
//   private final StatusSignal<Double> driveCurrent;

//   private final StatusSignal<Double> turnAbsolutePosition;
//   private final Queue<Double> turnPositionQueue;
//   private final StatusSignal<Double> turnVelocity;
//   private final StatusSignal<Double> turnAppliedVolts;
//   private final StatusSignal<Double> turnCurrent;

//   private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0);
//   private final VelocityVoltage velocityControl = new VelocityVoltage(0).withSlot(0).withUpdateFreqHz(0);
//   private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withSlot(0).withUpdateFreqHz(0);

//   public ModuleIOTalonFX(int driveMotorChannel,
//     int turnMotorChannel,
//     int turnEncoderChannel,
//     double angleZero,
//     SensorDirectionValue encoderReversed,
//     InvertedValue turnReversed,
//     InvertedValue driveReversed) {

//       // timestampQueue = new Queue<Double>() {};
      
//     driveTalon = new TalonFX(driveMotorChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
//     turnTalon = new TalonFX(turnEncoderChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
//     cancoder = new CANcoder(turnEncoderChannel, HardwareConstants.CANIVORE_CAN_BUS_STRING);
    
//     driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
//     driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//     driveTalon.getConfigurator().apply(driveConfig);
//     setDriveBrakeMode(true);

//     turnConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
//     turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//     turnTalon.getConfigurator().apply(turnConfig);
//     setTurnBrakeMode(true);

//     CANcoderConfiguration encoderConfig =  new CANcoderConfiguration();
//     encoderConfig.MagnetSensor.MagnetOffset = -angleZero;
//     cancoder.getConfigurator().apply(encoderConfig);

//     // timestampQueue = PhoenixOdometryThread.getInstance().;

//     drivePosition = driveTalon.getPosition();
//     drivePositionQueue =
//         PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
//     driveVelocity = driveTalon.getVelocity();
//     driveAppliedVolts = driveTalon.getMotorVoltage();
//     driveCurrent = driveTalon.getSupplyCurrent();

//     turnAbsolutePosition = cancoder.getAbsolutePosition();
//     turnPositionQueue =
//         PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
//     turnVelocity = turnTalon.getVelocity();
//     turnAppliedVolts = turnTalon.getMotorVoltage();
//     turnCurrent = turnTalon.getSupplyCurrent();

//     BaseStatusSignal.setUpdateFrequencyForAll(
//         HardwareConstants.SIGNAL_FREQUENCY, drivePosition);
//     BaseStatusSignal.setUpdateFrequencyForAll(
//         100,
//         driveVelocity,
//         driveAppliedVolts,
//         driveCurrent,
//         turnAbsolutePosition,
//         turnVelocity,
//         turnAppliedVolts,
//         turnCurrent);
//     driveTalon.optimizeBusUtilization();
//     turnTalon.optimizeBusUtilization();
//   }

//   @Override
//   public void updateInputs(ModuleIOInputs inputs) {
//     BaseStatusSignal.refreshAll(
//         drivePosition,
//         driveVelocity,
//         driveAppliedVolts,
//         driveCurrent,
//         turnAbsolutePosition,
//         turnVelocity,
//         turnAppliedVolts,
//         turnCurrent);

//     inputs.drivePositionMeters =
//         Units.rotationsToRadians(drivePosition.getValueAsDouble()) / ModuleConstants.DRIVE_GEAR_RATIO;
//     inputs.driveVelocityMetersPerSec =
//         Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / ModuleConstants.DRIVE_GEAR_RATIO;
//     inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
//     inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

//     inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
//     inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
//     inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

//     // inputs.odometryTimestamps =
//     //     timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
//     inputs.odometryDrivePositionsRad =
//         drivePositionQueue.stream()
//             .mapToDouble((Double value) -> Units.rotationsToRadians(value) / ModuleConstants.DRIVE_GEAR_RATIO)
//             .toArray();
//     // timestampQueue.clear();
//     drivePositionQueue.clear();
//     turnPositionQueue.clear();
//   }

//   @Override
//   public void setDriveVoltage(double volts) {
//     driveTalon.setControl(new VoltageOut(volts));
//   }

//   @Override
//   public void setTurnVoltage(double volts) {
//     turnTalon.setControl(new VoltageOut(volts));
//   }

//   @Override
//   public void setDriveBrakeMode(boolean enable) {
//     MotorOutputConfigs config = new MotorOutputConfigs();
//     config.Inverted = InvertedValue.CounterClockwise_Positive;
//     config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
//     driveTalon.getConfigurator().apply(config);
//   }

//   @Override
//   public void setTurnBrakeMode(boolean enable) {
//     MotorOutputConfigs config = new MotorOutputConfigs();
//     config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
//     turnTalon.getConfigurator().apply(config);
//   }

//   @Override
//   public void setDrivePID(double kP, double kI, double kD) {
//     driveConfig.Slot0.kP = kP;
//     driveConfig.Slot0.kI = kI;
//     driveConfig.Slot0.kD = kD;
//     driveTalon.getConfigurator().apply(driveConfig);
//   }

//   @Override
//   public void setTurnPID(double kP, double kI, double kD) {
//     turnConfig.Slot0.kP = kP;
//     turnConfig.Slot0.kI = kI;
//     turnConfig.Slot0.kD = kD;
//     turnTalon.getConfigurator().apply(turnConfig);
//   }

//   @Override
//   public void setDriveFF(double kS, double kV, double kA) {
//     driveConfig.Slot0.kS = kS;
//     driveConfig.Slot0.kV = kV;
//     driveConfig.Slot0.kA = kA;
//     driveTalon.getConfigurator().apply(driveConfig);
//   }

//   @Override
//   public void stop () {
//     driveTalon.setControl(neutralControl);
//     turnTalon.setControl(neutralControl);
//   }

//   public double getTurnRadians() {
//     turnAbsolutePosition.refresh();
//     return Rotation2d.fromRotations(turnAbsolutePosition.getValue()).getRadians();
//   }

//   @Override
//   public void setDesiredState(SwerveModuleState desiredState) {
//     double turnRadians = getTurnRadians();
 
//     // Optimize the reference state to avoid spinning further than 90 degrees
//     SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(turnRadians));

//     if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.01) {
//       driveTalon.set(0);
//       turnTalon.set(0);
//       return;
//     }

//     // Converts meters per second to rotations per second
//     double desiredDriveRPS = optimizedDesiredState.speedMetersPerSecond 
//      * ModuleConstants.DRIVE_GEAR_RATIO / ModuleConstants.WHEEL_CIRCUMFERENCE_METERS;
     
//     driveTalon.setControl(velocityControl.withVelocity(desiredDriveRPS));
//     turnTalon.setControl(positionControl.withPosition(optimizedDesiredState.angle.getRotations()));
//   }

// }