package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    io.setPivotPosition(angle);
    Logger.recordOutput("OTBIntake/Pivot", angle);
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

  public void setPivotSpeed(double speed) {
    io.setPivotSpeed(speed);
  }

  public double getPivotPosition() {
    return io.getPivotPosition();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("OTBIntake", inputs);

    Logger.recordOutput("pivot pos", getPivotPosition());
  }
}
