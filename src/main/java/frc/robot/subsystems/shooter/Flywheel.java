package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelInterface FlywheelInterface;
  private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();

  // private final SimpleMotorFeedforward ffModel;

  public Flywheel(FlywheelInterface io) {
    this.flywheelInterface = flywheelInterface;
  }

  /**
   * Sets the flywheel voltage
   *
   * @param desiredVoltage
   */
  public void setFlywheelVoltage(double desiredVoltage) {
    flywheelInterface.setVoltage(desiredVoltage); // /flywheelInterface calls the functions
    Logger.recordOutput("Flywheel/voltage", desiredVoltage);
  }

  /**
   * sets flywheel velocity in RPM
   *
   * @param desiredRPM desired velocity in RPM
   */
  public void setFlywheelVelocity(double desiredRPM) {
    flywheelInterface.setVelocity(desiredRPM);
    Logger.recordOutput("Flywheel/RPM", desiredRPM);
  }

  public boolean hasNote() {
    return inputs.isNoteDetected;
  }

  /**
   * sets the roller speed on the shooter
   *
   * @param speed desired speed in rotations/sec
   */
  public void setRollerSpeed(double speed) {
    flywheelInterface.setRollerSpeed(speed);
    Logger.recordOutput("Roller/DutyCycleOut", speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    flywheelInterface.updateInputs(inputs);
    Logger.processInputs("FlywheelSubsystem", inputs);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    flywheelInterface.updateInputs(inputs);
  }
}
