package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.HardwareConstants;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX indexerMotor = new TalonFX(0);

  private final StatusSignal<Double> indexerVelocity;
  private final StatusSignal<Double> indexerAppliedVolts;
  private final StatusSignal<Double> indexerStatorCurrentAmps;
  private final StatusSignal<Double> indexerSupplyCurrentAmps;
  private final StatusSignal<Double> indexerTemp;

  public IndexerIOTalonFX() {
    TalonFXConfiguration indexerConfiguration = new TalonFXConfiguration();
    indexerConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    indexerConfiguration.MotorOutput.DutyCycleNeutralDeadband =
        HardwareConstants.MIN_FALCON_DEADBAND;
    indexerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    indexerConfiguration.CurrentLimits.StatorCurrentLimit = 0.0;
    indexerConfiguration.CurrentLimits.SupplyCurrentLimit = 0.0;
    indexerConfiguration.CurrentLimits.StatorCurrentLimitEnable = false;
    indexerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = false;

    indexerMotor.getConfigurator().apply(indexerConfiguration);

    indexerVelocity = indexerMotor.getVelocity();
    indexerAppliedVolts = indexerMotor.getMotorVoltage();
    indexerStatorCurrentAmps = indexerMotor.getStatorCurrent();
    indexerSupplyCurrentAmps = indexerMotor.getSupplyCurrent();
    indexerTemp = indexerMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        indexerVelocity,
        indexerAppliedVolts,
        indexerStatorCurrentAmps,
        indexerSupplyCurrentAmps,
        indexerTemp);
    indexerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerAppliedVolts = indexerAppliedVolts.getValueAsDouble();
    inputs.indexerStatorCurrentAmps = indexerStatorCurrentAmps.getValueAsDouble();
    inputs.indexerSupplyCurrentAmps = indexerSupplyCurrentAmps.getValueAsDouble();
    inputs.indexerTemp = indexerTemp.getValueAsDouble();
    inputs.indexerVelocity = indexerVelocity.getValueAsDouble();
    inputs.isConnected =
        BaseStatusSignal.isAllGood(
            indexerVelocity,
            indexerAppliedVolts,
            indexerStatorCurrentAmps,
            indexerSupplyCurrentAmps,
            indexerTemp);
  }

  @Override
  public void setSpeed(double speed) {
    indexerMotor.set(speed);
  }

  @Override
  public void stop() {
    indexerMotor.stopMotor();
  }
}
