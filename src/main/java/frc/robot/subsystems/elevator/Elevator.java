package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorInterface elevatorInterface;
  private final ElevatorInterfaceInputsAutoLogged inputs = new ElevatorInterfaceInputsAutoLogged();

  public Elevator(ElevatorInterface io) {
    super("Elevator");
    this.elevatorInterface = elevatorInterface;
  }

  /**
   * sets the position of the elevator
   *
   * @param position position of elevator, in meters from minimum elevator height to max height
   */
  public void setElevatorPosition(double position) {
    Logger.recordOutput("Elevator", position);
    elevatorInterface.setElevatorPosition(position);
  }

  /**
   * Sets the speed of the elevator
   *
   * @param speed output value from -1.0 to 1.0
   */
  public void setElevatorSpeed(double speed) {
    elevatorInterface.setElevatorSpeed(speed);
  }

  @Override
  public void periodic() {
    elevatorInterface.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }
}
