package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerInterface indexerInterface;
  private IndexerInterfaceInputsAutoLogged inputs = new IndexerInterfaceInputsAutoLogged();

  public Indexer(IndexerInterface indexerInterface) {
    this.indexerInterface = indexerInterface;
  }

  /**
   * Sets the indexer speed
   *
   * @param speed The desired speed (-1.0 to 1.0)
   */
  public void setIndexerSpeed(double speed) {
    indexerInterface.setSpeed(speed);
    Logger.recordOutput("Indexer", speed);
  }

  /** Stops the motor */
  public void stop() {
    indexerInterface.stop();
  }

  @Override
  public void periodic() {
    indexerInterface.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }
}
