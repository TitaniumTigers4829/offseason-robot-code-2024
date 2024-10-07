// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.extras.interpolators.SingleLinearInterpolator;


/** Add your docs here. */
public class PivotIOTalon implements PivotIO {

    private final TalonFX leaderPivotMotor;
    private final TalonFX followerPivotMotor;

    private final CANcoder pivotEncoder;

    private final StatusSignal<Double> leaderPosition;
    private final StatusSignal<Double> leaderVelocity;
    private final StatusSignal<Double> leaderAppliedVolts;
    private final StatusSignal<Double> leaderSupplyCurrent;
    private final StatusSignal<Double> leaderTorqueCurrent;
    private final StatusSignal<Double> leaderTempCelsius;
    private final StatusSignal<Double> followerPosition;
    private final StatusSignal<Double> followerVelocity;
    private final StatusSignal<Double> followerAppliedVolts;
    private final StatusSignal<Double> followerSupplyCurrent;
    private final StatusSignal<Double> followerTorqueCurrent;
    private final StatusSignal<Double> followerTempCelsius;
    private final StatusSignal<Double> pivotPos;

    
    private final MotionMagicVoltage mmPositionRequest;

    private final SingleLinearInterpolator speakerAngleLookupValues;
    private final SingleLinearInterpolator passAngleLookupValues;

    private double pivotTargetAngle;

    private final Slot0Configs controllerConfig = new Slot0Configs();
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

    public PivotIOTalon() {
        leaderPivotMotor = new TalonFX(PivotConstants.LEADER_PIVOT_MOTOR_ID);
        followerPivotMotor = new TalonFX(PivotConstants.FOLLOWER_PIVOT_MOTOR_ID);
        pivotEncoder = new CANcoder(PivotConstants.PIVOT_ENCODER_ID);
        mmPositionRequest = new MotionMagicVoltage(0);
        speakerAngleLookupValues = new SingleLinearInterpolator(PivotConstants.SPEAKER_PIVOT_POSITION);
        passAngleLookupValues = new SingleLinearInterpolator(PivotConstants.PASS_PIVOT_POSITION);
    
        CANcoderConfiguration pivotEncoderConfig = new CANcoderConfiguration();
        pivotEncoderConfig.MagnetSensor.MagnetOffset = -PivotConstants.ANGLE_ZERO;
        pivotEncoderConfig.MagnetSensor.SensorDirection = PivotConstants.ENCODER_REVERSED;
        pivotEncoder.getConfigurator().apply(pivotEncoderConfig, HardwareConstants.TIMEOUT_S);


        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        pivotConfig.Slot0.kP = PivotConstants.PIVOT_P;
        pivotConfig.Slot0.kI = PivotConstants.PIVOT_I;
        pivotConfig.Slot0.kD = PivotConstants.PIVOT_D;
        pivotConfig.Slot0.kG = PivotConstants.PIVOT_G;

        pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        pivotConfig.MotionMagic.MotionMagicAcceleration =
            PivotConstants.MAX_VELOCITY_ROTATIONS_PER_SECOND;
        pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
            PivotConstants.MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED;

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;

        pivotConfig.ClosedLoopGeneral.ContinuousWrap = true;

        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        pivotConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();

        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.MAX_ANGLE;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.MIN_ANGLE;
        pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        leaderPivotMotor.getConfigurator().apply(pivotConfig, HardwareConstants.TIMEOUT_S);
        pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        followerPivotMotor.getConfigurator().apply(pivotConfig, HardwareConstants.TIMEOUT_S);


  
        leaderPosition = leaderPivotMotor.getPosition();
        leaderVelocity = leaderPivotMotor.getVelocity();
        leaderAppliedVolts = leaderPivotMotor.getMotorVoltage();
        leaderSupplyCurrent = leaderPivotMotor.getSupplyCurrent();
        leaderTorqueCurrent = leaderPivotMotor.getTorqueCurrent();
        leaderTempCelsius = leaderPivotMotor.getDeviceTemp();
        followerPosition = followerPivotMotor.getPosition();
        followerVelocity = followerPivotMotor.getVelocity();
        followerAppliedVolts = followerPivotMotor.getMotorVoltage();
        followerSupplyCurrent = followerPivotMotor.getSupplyCurrent();
        followerTorqueCurrent = followerPivotMotor.getTorqueCurrent();
        followerTempCelsius = followerPivotMotor.getDeviceTemp();
        
        
        pivotPos = pivotEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            leaderPosition,
            leaderVelocity,
            leaderAppliedVolts,
            leaderSupplyCurrent,
            leaderTorqueCurrent,
            leaderTempCelsius,
            followerPosition,
            followerVelocity,
            followerAppliedVolts,
            followerSupplyCurrent,
            followerTorqueCurrent,
            followerTempCelsius,
            pivotPos);
    }

    /**
     * Updates Inputs
     * @param inputs inputs for logging
     */
    @Override
    public void updateInputs(PivotIOInputs inputs) {
        
        BaseStatusSignal.refreshAll(
            leaderPosition,
            leaderVelocity,
            leaderAppliedVolts,
            leaderSupplyCurrent,
            leaderTorqueCurrent,
            leaderTempCelsius,
            followerPosition,
            followerVelocity,
            followerAppliedVolts,
            followerSupplyCurrent,
            followerTorqueCurrent,
            followerTempCelsius,
            pivotPos);
      
        inputs.leaderPosition = leaderPivotMotor.getPosition().getValueAsDouble();
        inputs.leaderVelocity = leaderPivotMotor.getVelocity().getValueAsDouble();
        inputs.leaderAppliedVolts = leaderPivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.leaderSupplyCurrentAmps = leaderPivotMotor.getSupplyCurrent().getValueAsDouble();

        inputs.followerPosition = followerPivotMotor.getPosition().getValueAsDouble();
        inputs.followerVelocity = followerPivotMotor.getVelocity().getValueAsDouble();
        inputs.followerAppliedVolts = followerPivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.followerSupplyCurrentAmps = followerPivotMotor.getSupplyCurrent().getValueAsDouble();
    }

    /**
     * Sets the voltage of the pivot motors
     *
     * @param volts the voltage
     */    
    @Override
    public void setVoltage(double volts) {
      leaderPivotMotor.setControl(new VoltageOut(volts));
      followerPivotMotor.setControl(new VoltageOut(volts));

    }


  /**
   * Gets the angle of the pivot
   *
   * @return angle of pivot in rotations
   */
    @Override
    public double getAngle() {
        pivotPos.refresh();
        return pivotPos.getValueAsDouble();
    }
    
    /**
     * Returns if the pivot is within an acceptable rotation in relation to the target position
     *
     * @return pivot error between desired and actual state in rotations
     */
    @Override
    public boolean isPivotWithinAcceptableError() {
        return Math.abs(pivotTargetAngle - getAngle()) < PivotConstants.PIVOT_ACCEPTABLE_ERROR;
    }

    /**
     *  Sets the output of the pivot
     * 
     * @param output output value from -1.0 to 1.9
     */
    @Override
    public void setPivotSpeed(double output) {
        leaderPivotMotor.set(output);
        followerPivotMotor.set(output);
      }

    /**
     * Gets the target angle of the pivot in degrees
     *
     * @return the target angle
     */
    @Override
    public double getPivotTarget() {
        return pivotTargetAngle;
      }

    /**
     * Uses distance in meters from the speaker to set the pivot angle (degrees) of the shooter
     *
     * @param speakerDistance the distance in meters from the speaker
     */
    @Override
    public void setPivotFromSpeakerDistance(double speakerDistance) {
        double speakerAngle = speakerAngleLookupValues.getLookupValue(speakerDistance);
        pivotTargetAngle = speakerAngle;
        setPivotAngle(speakerAngle);
      }

    /** Sets Pivot based on Pass Distance */
    @Override
    public void setPivotFromPassDistance(double passDistance) {
        double passAngle = passAngleLookupValues.getLookupValue(passDistance);
        pivotTargetAngle = passAngle;
        setPivotAngle(passAngle);
      }

    /**
     * Uses distance in meters from the passing position to set the pivot angle (degrees) of the
     * shooter
     *
     * @param passDistance the distance in meters from the passing position
     */
    @Override
    public void setPivotAngle(double angle) {
        pivotTargetAngle = angle;
        leaderPivotMotor.setControl(mmPositionRequest.withPosition(angle));
        followerPivotMotor.setControl(mmPositionRequest.withPosition(angle));
      }
}
