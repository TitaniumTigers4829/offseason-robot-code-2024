package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class ElevatorIOSim implements ElevatorIO {
  ElevatorSim elevatorSim;
  LinearSystem<N2, N1, N2> elevatorSystem;
  private Voltage elevatorAppliedVolts = Volts.zero();
  private final PIDController elevatorPID = new PIDController(0.0, 0.0, 0.0);

  public ElevatorIOSim() {
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
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02);

    inputs.isConnected = true;

    inputs.leaderMotorAppliedVolts = elevatorAppliedVolts.in(Volts);
    inputs.leaderMotorCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.leaderMotorPosition = metersToRotations(Meters.of(elevatorSim.getPositionMeters())).in(Meters);
    inputs.leaderMotorVelocity = metersPerSecondToRotations(MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond())).in(MetersPerSecond);

    inputs.followerMotorAppliedVolts = elevatorAppliedVolts.in(Volts);
    inputs.followerMotorCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.followerMotorPosition = metersToRotations(Meters.of(elevatorSim.getPositionMeters())).in(Meters);
    inputs.followerMotorVelocity = metersPerSecondToRotations(MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond())).in(MetersPerSecond);
  }

  private Distance metersToRotations(Distance value) {
    return (value.divide(Meters.of(2 * Math.PI * ElevatorConstants.DRUM_RADIUS).in(Meters))).times(ElevatorConstants.ELEVATOR_GEAR_RATIO);
  }

  private LinearVelocity metersPerSecondToRotations(LinearVelocity value) {
    return (value.divide(Meters.of(2 * Math.PI * ElevatorConstants.DRUM_RADIUS).in(Meters))).times(ElevatorConstants.ELEVATOR_GEAR_RATIO);
  }

  @Override
  public void setVolts(double volts) {
    elevatorAppliedVolts = Volts.of(volts);
    elevatorSim.setInputVoltage(volts);
  }

  @Override
  public double getVolts() {
    return elevatorAppliedVolts.in(Volts);
  }

  // @Override
  public void setElevatorPosition(double position) {
    elevatorAppliedVolts = Volts.of(elevatorPID.calculate(Meters.of(elevatorSim.getPositionMeters()).in(Meters), Meters.of(position).in(Meters)));

    if (elevatorSim.wouldHitUpperLimit(position)) {
      elevatorAppliedVolts = Volts.of(MathUtil.clamp((elevatorAppliedVolts).in(Volts), -12, 0));
    } else if (elevatorSim.wouldHitLowerLimit(position)) {
      elevatorAppliedVolts = Volts.of(MathUtil.clamp(elevatorAppliedVolts.in(Volts), 0, 12));
    } else {
      elevatorAppliedVolts = Volts.of(MathUtil.clamp(elevatorAppliedVolts.in(Volts),-12 ,12));
    }
    elevatorSim.setInputVoltage(elevatorAppliedVolts.in(Volts));
    
  }

  @Override
  public double getElevatorPosition() {
    return elevatorSim.getPositionMeters();
  }
}
