package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulatedIndexer implements IndexerInterface {
  private final DCMotorSim indexerSim = new DCMotorSim(null, DCMotor.getFalcon500(1), 0);

  private double indexerAppliedVolts = 0.0;

  public SimulatedIndexer(IndexerInputs inputs) {
    indexerSim.update(0.02);

    inputs.isConnected = true;

    inputs.indexerVelocity = indexerSim.getAngularVelocityRadPerSec() / (Math.PI * 2);
    inputs.indexerStatorCurrentAmps = indexerSim.getCurrentDrawAmps();
    inputs.indexerAppliedVolts = indexerAppliedVolts;
  }

  @Override
  public void setSpeed(double speed) {
    indexerSim.setInputVoltage(speed);
  }

  @Override
  public void stop() {
    indexerSim.setInputVoltage(0);
  }
}
