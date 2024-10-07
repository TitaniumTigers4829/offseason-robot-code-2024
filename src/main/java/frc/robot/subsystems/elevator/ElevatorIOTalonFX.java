package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.HardwareConstants;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leaderElevatorMotor;
    private final TalonFX followerElevatorMotor;

    private final MotionMagicVoltage mmPositionRequest;
    private final VoltageOut voltageRequest;
    private final NeutralOut neutralOut;


    StatusSignal<Double> leaderPosition;
    StatusSignal<Double> leaderVelocity;
    StatusSignal<Double> leaderAppliedVolts;
    StatusSignal<Double> leaderCurrentAmps;

    StatusSignal<Double> followerPosition;
    StatusSignal<Double> followerVelocity;
    StatusSignal<Double> followerAppliedVolts;
    StatusSignal<Double> followerCurrentAmps;

    public ElevatorIOTalonFX() {
        leaderElevatorMotor = new TalonFX(0);
        followerElevatorMotor = new TalonFX(0);

        mmPositionRequest = new MotionMagicVoltage(0.0);
        voltageRequest = new VoltageOut(0.0);
        neutralOut = new NeutralOut();

        TalonFXConfiguration elevatorConfiguration = new TalonFXConfiguration();
        elevatorConfiguration.Slot0.kP = 0.0;
        elevatorConfiguration.Slot0.kI = 0.0;
        elevatorConfiguration.Slot0.kD = 0.0;
        elevatorConfiguration.Slot0.kS = 0.0;
        elevatorConfiguration.Slot0.kV = 0.0;
        elevatorConfiguration.Slot0.kA = 0.0;
        elevatorConfiguration.Slot0.kG = 0.0;
        elevatorConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        elevatorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0.0;
        elevatorConfiguration.MotionMagic.MotionMagicAcceleration = 0.0;

        // TODO: TUNE!!!!
        elevatorConfiguration.CurrentLimits.StatorCurrentLimit = 0.0;
        elevatorConfiguration.CurrentLimits.StatorCurrentLimitEnable = false;
        elevatorConfiguration.CurrentLimits.SupplyCurrentLimit = 0.0;
        elevatorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = false;

        elevatorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // elevatorConfiguration.Feedback.FeedbackRotorOffset = 0.0;

        elevatorConfiguration.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
        elevatorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
        elevatorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
        elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        elevatorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        elevatorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;


        leaderElevatorMotor.getConfigurator().apply(elevatorConfiguration, HardwareConstants.TIMEOUT_S);
        followerElevatorMotor.getConfigurator().apply(elevatorConfiguration, HardwareConstants.TIMEOUT_S);

        leaderPosition = leaderElevatorMotor.getRotorPosition();
        leaderVelocity = leaderElevatorMotor.getRotorVelocity();
        leaderAppliedVolts = leaderElevatorMotor.getMotorVoltage();
        leaderCurrentAmps = leaderElevatorMotor.getStatorCurrent();

        followerPosition = followerElevatorMotor.getRotorPosition();
        followerVelocity = followerElevatorMotor.getRotorVelocity();
        followerAppliedVolts = followerElevatorMotor.getMotorVoltage();
        followerCurrentAmps = followerElevatorMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(HardwareConstants.SIGNAL_FREQUENCY, leaderPosition, leaderVelocity, followerPosition, followerVelocity);
        BaseStatusSignal.setUpdateFrequencyForAll(50, leaderAppliedVolts, leaderCurrentAmps, followerAppliedVolts, followerCurrentAmps);

        ParentDevice.optimizeBusUtilizationForAll(leaderElevatorMotor, followerElevatorMotor);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.isConnected = BaseStatusSignal.isAllGood(leaderPosition, leaderVelocity, followerPosition, followerVelocity, leaderAppliedVolts, leaderCurrentAmps, followerCurrentAmps);
        
        inputs.leaderMotorPosition = leaderPosition.getValueAsDouble();
        inputs.leaderMotorVelocity = leaderVelocity.getValueAsDouble();
        inputs.leaderMotorAppliedVolts = leaderAppliedVolts.getValueAsDouble();
        inputs.leaderMotorCurrentAmps= leaderCurrentAmps.getValueAsDouble();
        
        inputs.followerMotorPosition = followerPosition.getValueAsDouble();
        inputs.followerMotorVelocity = followerVelocity.getValueAsDouble();
        inputs.followerMotorAppliedVolts = followerAppliedVolts.getValueAsDouble();
        inputs.followerMotorCurrentAmps = followerCurrentAmps.getValueAsDouble();

    }

    @Override
    public void setElevatorPosition(double position) {
        leaderElevatorMotor.setControl(mmPositionRequest.withPosition(position));
        followerElevatorMotor.setControl(mmPositionRequest.withPosition(position));
    }

    @Override
    public double getElevatorPosition() {
        return leaderPosition.refresh().getValueAsDouble();
    }

    @Override
    public void setVolts(double volts) {
        leaderElevatorMotor.setControl(voltageRequest.withOutput(volts));
        followerElevatorMotor.setControl(voltageRequest.withOutput(volts));
    }

    
}
