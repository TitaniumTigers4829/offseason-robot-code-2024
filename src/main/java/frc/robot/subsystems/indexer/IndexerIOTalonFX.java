package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HardwareConstants;

public class IndexerIOTalonFX implements IndexerIO {
  private final TalonFX indexerMotor = new TalonFX(0);

  private final StatusSignal<AngularVelocity> indexerVelocity;
  private final StatusSignal<Voltage> indexerAppliedVolts;
  private final StatusSignal<Current> indexerStatorCurrentAmps;
  private final StatusSignal<Current> indexerSupplyCurrentAmps;
  private final StatusSignal<Temperature> indexerTemp;

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
