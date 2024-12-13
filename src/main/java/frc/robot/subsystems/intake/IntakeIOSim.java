package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import edu.wpi.first.units.measure.Angle;
import static edu.wpi.first.units.Units.*;

public class IntakeIOSim implements IntakeIO {
  DCMotorSim intakeSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), 0.01, 1), DCMotor.getFalcon500(1));
  SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(2), IntakeConstants.INTAKE_PIVOT_GEAR_RATIO, 0.01, 1, Rotations.of(0).in(Radians), Rotations.of(1).in(Radians), false, 0.0);
  private final Constraints pivotConstraints = new Constraints(5, 5);
  private final ProfiledPIDController pivotController =
      new ProfiledPIDController(20, 0, 0, pivotConstraints);

  private final ArmFeedforward pivotFF = new ArmFeedforward(0.1, 0, 0, 0);

  private double intakeAppliedVolts = 0.1;
  private double pivotAppliedVolts = 0.1;

  public IntakeIOSim() {
    
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeSim.update(0.02);
    pivotSim.update(0.02);
    inputs.intakeVelocity = RadiansPerSecond.of(intakeSim.getAngularVelocityRadPerSec()).in(RotationsPerSecond);
    inputs.intakeCurrentAmps = intakeSim.getCurrentDrawAmps();
    inputs.intakeAppliedVolts = intakeAppliedVolts;

    inputs.pivotPosition = Radians.of(pivotSim.getAngleRads()).in(Rotations);
    inputs.pivotVelocity = RadiansPerSecond.of(pivotSim.getVelocityRadPerSec()).in(RotationsPerSecond);
    inputs.pivotAppliedVolts = pivotAppliedVolts;
    inputs.pivotCurrentAmps = pivotSim.getCurrentDrawAmps();
  }

  @Override
  public void setPivotPosition(double position) {
    pivotSim.setInputVoltage(
        pivotController.calculate(Radians.of(pivotSim.getAngleRads()).in(Rotations), position));
            // + pivotFF.calculate(
            //     pivotController.getSetpoint().position, pivotController.getSetpoint().velocity));
  }

  @Override
  public void setIntakeSpeed(double speed) {
    intakeSim.setInputVoltage(speed);
  }

  @Override
  public double getIntakeSpeed() {
    return intakeSim.getAngularVelocityRadPerSec() / (Math.PI * 2);
  }

  @Override
  public double getPivotPosition() {
    return Radians.of(pivotSim.getAngleRads()).in(Rotations);
  }
}
