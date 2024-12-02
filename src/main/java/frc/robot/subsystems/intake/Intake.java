package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeInterface intakeInterface;
  private IntakeInterfaceInputsAutoLogged inputs = new IntakeInterfaceInputsAutoLogged();

  public Intake(IntakeInterface intakeInterface) {
    this.intakeInterface = intakeInterface;
  }

  /**
   * sets the angle of the pivot in degrees
   *
   * @param angle desired angle in degrees
   */
  public void setPivotAngle(double angle) {
    double angleRots = Units.degreesToRotations(angle);
    intakeInterface.setPivotPosition(angleRots);
    Logger.recordOutput("OTBIntake/Pivot", angleRots);
  }

  /**
   * Sets the speed of the intake rollers
   *
   * @param speed intake roller speed (-1.0 to 1.0)
   */
  public void setIntakeSpeed(double speed) {
    intakeInterface.setIntakeSpeed(speed);
    Logger.recordOutput("OTBIntake/Intake", speed);
  }

  public void setPivotSpeed(double speed) {
    intakeInterface.setPivotSpeed(speed);
  }

  public void getPivotPosition() {
    intakeInterface.getPivotPosition();
  }

  @Override
  public void periodic() {
    intakeInterface.updateInputs(inputs);
    Logger.processInputs("OTBIntake", inputs);
  }
}
