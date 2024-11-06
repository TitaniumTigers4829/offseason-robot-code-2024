package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim
    implements FlywheelIO { // FlywheelIOSim makes Advantage kit log simulated movements using
  // physics
  private FlywheelSim flywheelSim = new FlywheelSim(null, DCMotor.getKrakenX60(2), 0);
  private DIOSim noteSensorSim = new DIOSim(new DigitalInput(ShooterConstants.NOTE_SENSOR_ID));
  private PIDController pid =
      new PIDController(
          ShooterConstants.FLYWHEEL_P, ShooterConstants.FLYWHEEL_I, ShooterConstants.FLYWHEEL_D);
  private SimpleMotorFeedforward feedForward =
      new SimpleMotorFeedforward(
          ShooterConstants.FLYWHEEL_S, ShooterConstants.FLYWHEEL_V, ShooterConstants.FLYWHEEL_A);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    flywheelSim.update(0.02);

    inputs.positionRotations = 0.0;
    inputs.velocityRPM = flywheelSim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {flywheelSim.getCurrentDrawAmps()};
    inputs.isNoteDetected = hasNote();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    flywheelSim.setInputVoltage(volts);
  }


  @Override
  public void setVelocity(double velocityRPM) { // ffvolts is feedorward
    double velocityRPS = velocityRPM / 60;
    flywheelSim.setInput(pid.calculate(velocityRPS) + feedForward.calculate(velocityRPS));
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  public boolean hasNote() {
    return !noteSensorSim.getValue();
  }
}
