package frc.robot.extras.simulation;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class OdometryTimestampsSim {
  public static double[] getTimeStamps() {
    final double[] odometryTimestamps = new double[5];
    for (int i = 0; i < 5; i++)
      odometryTimestamps[i] =
          Timer.getFPGATimestamp() - Robot.defaultPeriodSecs + SimulatedField.SIMULATION_DT * i;
    return odometryTimestamps;
  }
}
