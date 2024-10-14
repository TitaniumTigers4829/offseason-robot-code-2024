package frc.robot.subsystems.swerve.gyroIO;

import frc.robot.extras.simulation.OdometryTimestampsSim;
import frc.robot.extras.simulation.physicsSim.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = true;
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    inputs.odometryYawTimestamps = OdometryTimestampsSim.getTimeStamps();
    inputs.yawDegrees = gyroSimulation.getGyroReading();
    inputs.yawVelocity = gyroSimulation.getMeasuredAngularVelocityRadPerSec();
  }
}
