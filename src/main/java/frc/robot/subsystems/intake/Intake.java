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
   * sets the angle of the pivot in degrees
   *
   * @param angle desired angle in degrees
   */
  public void setPivotAngle(double angle) {
    double angleRots = angle / 360;
    io.setPivotPosition(angleRots);
    Logger.recordOutput("OTBIntake/Pivot", angleRots);
  }

  /**
   * Sets the speed of the intake rollers
   *
   * @param speed intake roller speed (-1.0 to 1.0)
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
