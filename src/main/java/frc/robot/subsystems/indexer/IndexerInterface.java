package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerInterface {
  @AutoLog
  public static class IndexerInputs {
    public boolean isConnected = false;
    public double indexerVelocity = 0.0;
    public double indexerAppliedVolts = 0.0;
    public double indexerTemp = 0.0;
    public double indexerStatorCurrentAmps = 0.0;
    public double indexerSupplyCurrentAmps = 0.0;
  }

  default void updateInputs(IndexerInputs inputs) {}

  default boolean isIndexing() {
    return false;
  }

  default void setSpeed(double speed) {}

  default void stop() {}
}
