package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    super("Elevator");
    this.io = io;
  }

  /**
   * sets the position of the elevator
   *
   * @param position position of elevator, in meters from minimum elevator height to max height
   */
  public void setElevatorPosition(double position) {
    Logger.recordOutput("Elevator", position);
    io.setElevatorPosition(position);
  }

  /**
   * Sets the speed of the elevator
   *
   * @param speed output value from -1.0 to 1.0
   */
  public void setElevatorSpeed(double speed) {
    io.setElevatorSpeed(speed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }
}
