// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

package frc.robot.extras.purplelib;

import frc.robot.extras.purplelib.CANCoderInputsAutoLogged;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.signals.SensorDirectionValue;

/** CTRE CANCoder */
public class LoggedCANCoder extends LoggableHardware {

  /** CANCoder Status Frame */
  public enum CANCoderFrame {
    ABSOLUTE_POSITION,
    RELATIVE_POSITION,
    VELOCITY
  }

  @AutoLog
  public static class CANCoderInputs {
    public double absolutePosition = 0.0;
    public double relativePosition = 0.0;
    public double velocity = 0.0;
  }

  private CANcoder m_canCoder;

  private CANCoderInputsAutoLogged m_inputs;
  private String m_encoderName;

  public LoggedCANCoder(int deviceID, String busName, String encoderName) {
    this.m_canCoder = new CANcoder(deviceID, busName);
    this.m_inputs = new CANCoderInputsAutoLogged();
    this.m_encoderName = encoderName;

    // Update inputs on init
    periodic();

    // Register device with manager
    PurpleManager.add(this);
  }

   /**
     * Gets the absolute position of the sensor.
     * The absolute position may be unsigned (for example: [0,360) deg), or signed (for example: [-180,+180) deg). This is determined by a configuration. The default selection is unsigned.
     * The units are determined by the internal coefficient, default is rotations.
     * Note: this signal is not affected by calls to SetPosition().
     * @return The position of the sensor.
     */
  public StatusSignal<Double> getAbsolutePosition() {
    m_canCoder.getAbsolutePosition().refresh();
    return m_canCoder.getAbsolutePosition();
  }

  /**
   * Gets the position of the sensor.  This may be relative or absolute depending on configuration.
   * The units are determined by the internal coefficient, default is rotations.
   * @return The position of the sensor.
   */
  private double getRelativePosition() {
    m_canCoder.getPosition().refresh();
    return m_canCoder.getPosition().getValue();
  }

  /**
   * Gets the velocity of the sensor.
   * The units are determined by the internal coefficient, default is rotations per second.
   * @return The velocity of the sensor.
   */
  private double getVelocity() {
    m_canCoder.getVelocity().refresh();
    return m_canCoder.getVelocity().getValue();
  }

  /**
   * Gets the velocity of the sensor.
   * The units are determined by the internal coefficient, default is rotations.
   * @return The Velocity of the sensor.
   */
  private void updateInputs() {
    m_inputs.absolutePosition = getAbsolutePosition().getValue();
    m_inputs.relativePosition = getRelativePosition();
    m_inputs.velocity = getVelocity();
  }

  public int getDeviceID() {
    return m_canCoder.getDeviceID();
  }
  

  /**
   * Call this method periodically
   */
  @Override
  protected void periodic() {
    updateInputs();
    Logger.processInputs(m_encoderName, m_inputs);
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public CANCoderInputsAutoLogged getInputs() {
    return m_inputs;
  }


  /**
   * Sets the position of the sensor to specified value
   * The units are determined by the coefficient and unit-string configuration params, default is rotations.
   * @param position Position to reset to
   * @return StatusCode generated by function. 0 indicates no error.
   */
  public StatusCode resetPosition(double position) {
    return m_canCoder.setPosition(position);
  }

 /**
   * Sets the position of the sensor to zero
   * The units are determined by the coefficient and unit-string configuration params, default is degrees.
   * @return StatusCode generated by function. 0 indicates no error.
   */
  public StatusCode resetPosition() {
    return resetPosition(0.0);
  }

  /**
   * Configures all persistent settings to defaults (overloaded so timeoutMs is 50 ms).
   *
   * @return Status Code generated by function. 0 indicates no error.
   */
  public StatusCode configFactoryDefault() {
    return m_canCoder.getConfigurator().apply(new CANcoderConfiguration());
  }

  public StatusCode applyConfigs(CANcoderConfiguration configs, double timeoutSeconds) {
    return m_canCoder.getConfigurator().apply(configs, timeoutSeconds);
  }

  /**
   * Configures direction of the sensor to determine positive facing the LED side of the CANcoder.
   * @param direction The new sensor direction
   * @return StatusCode generated by function. 0 indicates no error.
   */
  public StatusCode configSensorDirection(SensorDirectionValue direction) {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = direction;

    return m_canCoder.getConfigurator().apply(config);
  }

	/**
	 * Sets the period of the given status frame.
	 *
	 * @param statusFrame
	 *            Frame whose period is to be changed.
	 * @param frequencyHz
	 *            Frequency in Hz for the given frame.
	 * @return Status Code generated by function. 0 indicates no error.
	 */
  public StatusCode setStatusFramePeriod(CANCoderFrame statusFrame, int frequencyHz) {
    switch (statusFrame) {
      case ABSOLUTE_POSITION:
        return m_canCoder.getAbsolutePosition().setUpdateFrequency(frequencyHz);
      case RELATIVE_POSITION:
        return m_canCoder.getPosition().setUpdateFrequency(frequencyHz);
      case VELOCITY:
        return m_canCoder.getVelocity().setUpdateFrequency(frequencyHz);
      default:
        return StatusCode.OK;
    }
  }

  @Override
  public void close() {
    PurpleManager.remove(this);
    m_canCoder = null;
  }
}

