package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  /** Constructor that uses IO. IO methods can be used in subsystem. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  /**
   * Sets the flywheel voltage
   *
   * @param desiredVoltage Desired Voltage in double
   */
  public void setFlywheelVoltage(double desiredVoltage) {
    io.setVoltage(desiredVoltage); // /IO calls the functions
    Logger.recordOutput("Flywheel/voltage", desiredVoltage);
  }

  /**
   * sets flywheel velocity in RPM
   *
   * @param desiredRPM desired velocity in RPM
   */
  public void setFlywheelVelocity(double desiredRPM) {
    io.setVelocity(desiredRPM);
    Logger.recordOutput("Flywheel/RPM", desiredRPM);
  }

  /**
   * Returns whether or not a note is detected
   *
   * @return False if not detected, True if detected
   */
  public boolean hasNote() {
    return inputs.isNoteDetected;
  }

  /**
   * sets the roller speed on the shooter
   *
   * @param speed desired speed in rotations/sec
   */
  public void setRollerSpeed(double speed) {
    io.setRollerSpeed(speed);
    Logger.recordOutput("Roller/DutyCycleOut", speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    io.updateInputs(inputs);
  }
}
