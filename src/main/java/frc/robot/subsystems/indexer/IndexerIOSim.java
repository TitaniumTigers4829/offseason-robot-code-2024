package frc.robot.subsystems.indexer;

public class IndexerIOSim implements IndexerIO {
    private final DCMotorSim indexerSim = new DCMotor(null, 0, 0);

    private double indexerAppliedVolts = 0.0;

    public IndexerIOSim() {
        indexerSim.update(0.02);

        inputs.isConnected = true;

        inputs.indexerVelocity = intakeSim.getVelocityRadPerSec() / (Math.PI * 2); 
        inputs.indexerStatorCurrentAmps = intakeSim.getCurrentDrawAmps();
        inputs.indexerAppliedVolts = intakeAppliedVolts;
    }

    @Override 
    public void setIndexerSpeed(double speed) {
        indexerSim.setInputVoltage(speed);
    }

    @Override 
    public void stop() {
        indexerSim.setInputVoltage(0);
    }

}