package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setPivotAngle(double angle) {
    io.setPivotPosition(angle);
    Logger.recordOutput("OTBIntake/Pivot", angle);
  }

  public void setIntakeSpeed(double speed) {
    io.setIntakeSpeed(speed);
    Logger.recordOutput("OTBIntake/Intake", speed);
  }

  public void setPivotSpeed(double speed) {
    io.setPivotSpeed(speed);
  }

  public void getPivotPosition() {
    io.getPivotPosition();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("OTBIntake", inputs);
  }
}
