package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.HardwareConstants;

public class FlywheelIOTalonFX
    implements FlywheelIO { // FlywheelIOTalonFX makes Advantagekit log physical hardware movement
  private final TalonFX leftFlywheelMotor =
      new TalonFX(FlywheelConstants.LEFT_FLYWHEEL_MOTOR_ID); // Leader=left motor
  private final TalonFX rightFlywheelMotor =
      new TalonFX(FlywheelConstants.RIGHT_FLYWHEEL_MOTOR_ID); // follower = right motor
  private final DigitalInput noteSensor =
      new DigitalInput(FlywheelConstants.NOTE_SENSOR_ID); // Note sensor
  private final TalonFX rollerMotor = new TalonFX(FlywheelConstants.ROLLER_MOTOR_ID);
  // gives values for each thing that is set.
  private final VelocityVoltage velocityRequest;
  private final VoltageOut voltageRequest;
  private final StatusSignal<Angle> leaderPosition = leftFlywheelMotor.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = leftFlywheelMotor.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = leftFlywheelMotor.getMotorVoltage();
  private final StatusSignal<Current> leaderCurrent = leftFlywheelMotor.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent =
      rightFlywheelMotor
          .getSupplyCurrent(); // All of these are from TalonFX Phoenix 6 that assign values to

  // advantagekit log variables

  public FlywheelIOTalonFX() { // Object to set different flywheel configs
    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.CurrentLimits.SupplyCurrentLimit =
        FlywheelConstants
            .FLYWHEEL_SUPPLY_LIMIT; // Talonfx configuration software limits found in CONSTANTS file
    flywheelConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.FLYWHEEL_STATOR_LIMIT;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable =
        FlywheelConstants.FLYWHEEL_SUPPLY_ENABLE;
    flywheelConfig.CurrentLimits.StatorCurrentLimitEnable =
        FlywheelConstants.FLYWHEEL_STATOR_ENABLE;
    flywheelConfig.MotorOutput.NeutralMode =
        NeutralModeValue
            .Coast; // This is used to ensure maximum power efficiency, to ensure flywheels keep
    // spinning after power off.

    flywheelConfig.Slot0.kP =
        FlywheelConstants.FLYWHEEL_P; // everything tuned in Flywheel Constants file
    flywheelConfig.Slot0.kI = FlywheelConstants.FLYWHEEL_I;
    flywheelConfig.Slot0.kP = FlywheelConstants.FLYWHEEL_D;
    flywheelConfig.Slot0.kS = FlywheelConstants.FLYWHEEL_S;
    flywheelConfig.Slot0.kV = FlywheelConstants.FLYWHEEL_V;
    flywheelConfig.Slot0.kA = FlywheelConstants.FLYWHEEL_A;

    leftFlywheelMotor
        .getConfigurator()
        .apply(flywheelConfig); // Apply the flywheel config defined above
    flywheelConfig.MotorOutput.Inverted =
        InvertedValue.Clockwise_Positive; // invert one so that they can go in the same direction
    rightFlywheelMotor.getConfigurator().apply(flywheelConfig);

    TalonFXConfiguration rollerConfiguration = new TalonFXConfiguration();
    rollerConfiguration.MotorOutput.DutyCycleNeutralDeadband =
        HardwareConstants.MIN_FALCON_DEADBAND;
    rollerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerMotor.getConfigurator().apply(rollerConfiguration);

    // sets the frequency where the values are updated
    BaseStatusSignal.setUpdateFrequencyForAll(
        HardwareConstants.SIGNAL_FREQUENCY,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerCurrent);
    leftFlywheelMotor
        .optimizeBusUtilization(); // updates frequency based on how often the bus communicates with
    // the motor
    rightFlywheelMotor.optimizeBusUtilization();

    velocityRequest = new VelocityVoltage(0.0);
    voltageRequest = new VoltageOut(0.0);
  }

  @Override
  public void updateInputs(
      FlywheelIOInputs inputs) { // gets current values for motors and puts them into a log
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.positionRotations =
        leaderPosition.getValueAsDouble()
            / FlywheelConstants.GEAR_RATIO; // converted to radians to gear ratio math
    inputs.velocityRPM = (leaderVelocity.getValueAsDouble() * 60.0) / FlywheelConstants.GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {
          leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()
        }; // puts current values into an array as doubles
    inputs.isNoteDetected = hasNote();
  }

  @Override
  public void setVoltage(double volts) { // some method to set voltage for motor
    leftFlywheelMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(double velocityRPM) {
    leftFlywheelMotor.setControl(velocityRequest.withVelocity(velocityRPM / 60.0));
  }

  @Override
  public void stop() { // stops the motor
    leftFlywheelMotor.stopMotor();
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public boolean hasNote() {
    return !noteSensor.get();
  }
}
