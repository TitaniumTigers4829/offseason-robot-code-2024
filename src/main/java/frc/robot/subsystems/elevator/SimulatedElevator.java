package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class SimulatedElevator implements ElevatorInterface {
  ElevatorSim elevatorSim;
  LinearSystem<N2, N1, N2> elevatorSystem;
  private double elevatorAppliedVolts = 0.0;
  private final PIDController elevatorPID = new PIDController(0.0, 0.0, 0.0);

  public SimulatedElevator() {
    this.elevatorSystem = LinearSystemId.createElevatorSystem(DCMotor.getFalcon500(2), 0, 0, 0);
    this.elevatorSim =
        new ElevatorSim(
            elevatorSystem,
            DCMotor.getFalcon500(2),
            ElevatorConstants.MIN_HEIGHT,
            ElevatorConstants.MAX_HEIGHT,
            true,
            0);
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    elevatorSim.update(0.02);

    inputs.isConnected = true;

    inputs.leaderMotorAppliedVolts = elevatorAppliedVolts;
    inputs.leaderMotorCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.leaderMotorPosition = metersToRotations(elevatorSim.getPositionMeters());
    inputs.leaderMotorVelocity = metersToRotations(elevatorSim.getVelocityMetersPerSecond());

    inputs.followerMotorAppliedVolts = elevatorAppliedVolts;
    inputs.followerMotorCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.followerMotorPosition = metersToRotations(elevatorSim.getPositionMeters());
    inputs.followerMotorVelocity = metersToRotations(elevatorSim.getVelocityMetersPerSecond());
  }

  private double metersToRotations(double value) {
    return (value / (2 * Math.PI * ElevatorConstants.DRUM_RADIUS))
        * ElevatorConstants.ELEVATOR_GEAR_RATIO;
  }

  @Override
  public void setVolts(double volts) {
    elevatorAppliedVolts = volts;
    elevatorSim.setInputVoltage(volts);
  }

  @Override
  public double getVolts() {
    return elevatorAppliedVolts;
  }

  @Override
  public void setElevatorPosition(double position) {
    elevatorAppliedVolts = elevatorPID.calculate(elevatorSim.getPositionMeters(), position);

    if (elevatorSim.wouldHitUpperLimit(position)) {
      elevatorAppliedVolts = MathUtil.clamp(elevatorAppliedVolts, -12, 0);
    } else if (elevatorSim.wouldHitLowerLimit(position)) {
      elevatorAppliedVolts = MathUtil.clamp(elevatorAppliedVolts, 0, 12);
    } else {
      elevatorAppliedVolts = MathUtil.clamp(elevatorAppliedVolts, -12, 12);
    }

    elevatorSim.setInputVoltage(elevatorAppliedVolts);
  }

  @Override
  public double getElevatorPosition() {
    return elevatorSim.getPositionMeters();
  }
}
