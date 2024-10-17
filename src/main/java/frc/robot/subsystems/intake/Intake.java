package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  /**
   * sets the angle of the pivot in rotations
   *
   * @param angle desired angle in rotations
   */
  public void setPivotAngle(double angle) {
    io.setPivotPosition(angle);
    Logger.recordOutput("OTBIntake/Pivot", angle);
  }

  /**
   * sets the speed of the intake rollers
   *
   * @param speed sets the intake roller speed in rotations/sec
   */
  public void setIntakeSpeed(double speed) {
    io.setIntakeSpeed(speed);
    Logger.recordOutput("OTBIntake/Intake", speed);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("OTBIntake", inputs);
  }
}
