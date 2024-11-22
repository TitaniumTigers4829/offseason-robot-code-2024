package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean isConnected = false;

    //Leader Motor = Main focus on command//
    public double leaderMotorPosition = 0.0; //Position of Elevator//
    public double leaderMotorVelocity = 0.0;  // Speed of Elevator//
    public double leaderMotorAppliedVolts = 0.0; // AppliedVolts is new Volt//
    public double leaderMotorCurrentAmps = 0.0; // How much power currently position//

    //This follower focus mostly to Leader//
    public double followerMotorPosition = 0.0;
    public double followerMotorVelocity = 0.0;
    public double followerMotorAppliedVolts = 0.0;
    public double followerMotorCurrentAmps = 0.0;
  }
  /**
   * Updating input information of elevator
   * @param inputs 
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}
 /**
  * Getting Position of Elevator 
  * @return /Return the value position (very important for special people)
  */
  public default double getElevatorPosition() {
    return 0.0;
  }
/**
 * Getting the Value Position 
 * @param position
 */
  public default void setElevatorPosition(double position) {}
/**
 * Trying to setElevator to 
 * @param speed speed is the total velocity
 */
  public default void setElevatorSpeed(double speed) {}

  public default void setVolts(double volts) {}

  public default double getVolts() {
    return 0.0;
  }
}
