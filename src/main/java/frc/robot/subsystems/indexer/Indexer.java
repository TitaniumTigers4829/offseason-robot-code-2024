package frc.robot.subsystems.indexer;

public class Indexer extends SubsystemBase {
    private IndexerIOInputs io
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIOInputs io) {
        this.io = io;
    }

    public setIndexerSpeed(double speed) {
        io.setSpeed(speed);
        Logger.recordOutput("Indexer", speed);
    }

    public stop() {
        io.stop();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }
}