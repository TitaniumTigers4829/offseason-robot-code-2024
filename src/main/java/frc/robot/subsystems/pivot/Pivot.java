package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.interpolators.SingleLinearInterpolator;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotInterface pivotInterface;
  private final PivotInterfaceInputsAutoLogged inputs = new PivotInterfaceInputsAutoLogged();

  private final SingleLinearInterpolator speakerAngleLookupValues;
  private final SingleLinearInterpolator speakerOverDefenseAngleLookupValues;
  private final SingleLinearInterpolator passAngleLookupValues;

  /** Creates a new Pivot. */
  public Pivot(PivotInterface pivotInterface) {
    this.pivotInterface = pivotInterface;
    speakerAngleLookupValues = new SingleLinearInterpolator(PivotConstants.SPEAKER_PIVOT_POSITION);
    speakerOverDefenseAngleLookupValues =
        new SingleLinearInterpolator(PivotConstants.SPEAKER_OVER_DEFENSE_PIVOT_POSITION);
    passAngleLookupValues = new SingleLinearInterpolator(PivotConstants.PASS_PIVOT_POSITION);
  }

  /**
   * sets the pivot angle(rotations) shooter
   *
   * @param angle the desired angle in rotations
   */
  public void setPivotAngle(double angle) {
    pivotInterface.setPivotAngle(angle);
  }

  /**
   * Uses distance in meters from the speaker to set the pivot angle (degrees) of the shooter
   *
   * @param speakerDistance the distance in meters from the speaker
   */
  public void setPivotFromSpeakerDistance(double speakerDistance) {
    double speakerAngle = speakerAngleLookupValues.getLookupValue(speakerDistance);
    setPivotAngle(speakerAngle);
  }

  /**
   * Uses distance in meters from the passing position to set the pivot angle (degrees) of the
   * shooter
   *
   * @param passDistance the distance in meters from the passing position
   */
  public void setPivotFromPassDistance(double passDistance) {
    double passAngle = passAngleLookupValues.getLookupValue(passDistance);
    setPivotAngle(passAngle);
  }

  /**
   * Uses distance in meters from the speaker to set the pivot angle (degrees) of the shooter with
   * elevator at max height
   *
   * @param speakerDistance the distance in meters from the speaker
   */
  public void setPivotFromSpeakerDistanceOverDefense(double speakerDistance) {
    double speakerAngle = speakerOverDefenseAngleLookupValues.getLookupValue(speakerDistance);
    setPivotAngle(speakerAngle);
  }

  /**
   * Sets the output of the pivot
   *
   * @param output output value from -1.0 to 1.0
   */
  public void setPivotSpeed(double output) {
    pivotInterface.setPivotSpeed(output);
  }

  /**
   * Returns if the pivot is within an acceptable rotation in relation to the target position
   *
   * @return pivot error between desired and actual state in rotations
   */
  public boolean isPivotWithinAcceptableError() {
    return pivotInterface.isPivotWithinAcceptableError();
  }

  /**
   * Gets the angle of the pivot
   *
   * @return angle of pivot in rotations
   */
  public double getAngle() {
    return pivotInterface.getAngle();
  }

  /**
   * Gets the target angle of the pivot in degrees
   *
   * @return the target angle
   */
  public double getPivotTarget() {
    return pivotInterface.getPivotTarget();
  }

  /**
   * Sets the voltage of the pivot motors
   *
   * @param volts the voltage
   */
  public void runVolts(double volts) {
    pivotInterface.setVoltage(volts);
  }

  @Override
  public void periodic() {
    pivotInterface.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    pivotInterface.updateInputs(inputs);
  }
}
