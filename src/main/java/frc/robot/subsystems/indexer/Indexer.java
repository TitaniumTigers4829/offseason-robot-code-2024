package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private IndexerIO io;
  private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void setIndexerSpeed(double speed) {
    io.setSpeed(speed);
    Logger.recordOutput("Indexer", speed);
  }

  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
  }
}
