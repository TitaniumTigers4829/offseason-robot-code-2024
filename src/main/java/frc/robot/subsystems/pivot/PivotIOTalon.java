// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


/** Add your docs here. */
public class PivotIOTalon implements PivotIO {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    private final StatusSignal<Double> leftPosition;
    private final StatusSignal<Double> leftVelocity;
    private final StatusSignal<Double> leftAppliedVolts;
    private final StatusSignal<Double> leftSupplyCurrent;
    private final StatusSignal<Double> leftTorqueCurrent;
    private final StatusSignal<Double> leftTempCelsius;
    private final StatusSignal<Double> rightPosition;
    private final StatusSignal<Double> rightVelocity;
    private final StatusSignal<Double> rightAppliedVolts;
    private final StatusSignal<Double> rightSupplyCurrent;
    private final StatusSignal<Double> rightTorqueCurrent;
    private final StatusSignal<Double> rightTempCelsius;

    private final Slot0Configs controllerConfig = new Slot0Configs();
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

    public PivotIOTalon(){
        leftMotor = new TalonFX(1, "rio");
        rightMotor = new TalonFX(2, "rio");

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        controllerConfig.kP = 0;
        controllerConfig.kI = 0;
        controllerConfig.kD = 0;
        controllerConfig.kS = 0;
        controllerConfig.kV = 0;
        controllerConfig.kA = 0;

        leftMotor.getConfigurator().apply(config, 1.0);
        rightMotor.getConfigurator().apply(config, 1.0);
        leftMotor.getConfigurator().apply(controllerConfig, 1.0);
        rightMotor.getConfigurator().apply(controllerConfig, 1.0);

        leftPosition = leftMotor.getPosition();
        leftVelocity = leftMotor.getVelocity();
        leftAppliedVolts = leftMotor.getMotorVoltage();
        leftSupplyCurrent = leftMotor.getSupplyCurrent();
        leftTorqueCurrent = leftMotor.getTorqueCurrent();
        leftTempCelsius = leftMotor.getDeviceTemp();
        rightPosition = rightMotor.getPosition();
        rightVelocity = rightMotor.getVelocity();
        rightAppliedVolts = rightMotor.getMotorVoltage();
        rightSupplyCurrent = rightMotor.getSupplyCurrent();
        rightTorqueCurrent = rightMotor.getTorqueCurrent();
        rightTempCelsius = rightMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            leftPosition,
            leftVelocity,
            leftAppliedVolts,
            leftSupplyCurrent,
            leftTorqueCurrent,
            leftTempCelsius,
            rightPosition,
            rightVelocity,
            rightAppliedVolts,
            rightSupplyCurrent,
            rightTorqueCurrent,
            rightTempCelsius);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs){
        BaseStatusSignal.refreshAll(
            leftPosition,
            leftVelocity,
            leftAppliedVolts,
            leftSupplyCurrent,
            leftTorqueCurrent,
            leftTempCelsius,
            rightPosition,
            rightVelocity,
            rightAppliedVolts,
            rightSupplyCurrent,
            rightTorqueCurrent,
            rightTempCelsius);
    }

    @Override
    public void runVolts(double leftvolts){
        leftMotor.setControl(new VoltageOut(leftvolts));    
    }

    @Override
    public void runVelocity(double leftRpm, double rightRpm, double leftFeedforward, double rightFeedforward){
        leftMotor.setControl(new VelocityVoltage(leftRpm));
    }

    @Override
    public void setPID(double kP, double kI, double kD){
    }

    @Override
    public void runCharacterizationLeft(double input){

    }

    @Override
    public void runCharacterizationRight(double input){
        
    }
}
