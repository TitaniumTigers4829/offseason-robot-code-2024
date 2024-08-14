// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.


package frc.robot.extras.purplelib;


import frc.robot.extras.purplelib.TalonFXInputsAutoLogged;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.DifferentialConstantsConfigs;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DifferentialDutyCycle;
import com.ctre.phoenix6.controls.DifferentialFollower;
import com.ctre.phoenix6.controls.DifferentialMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.controls.DifferentialPositionDutyCycle;
import com.ctre.phoenix6.controls.DifferentialPositionVoltage;
import com.ctre.phoenix6.controls.DifferentialStrictFollower;
import com.ctre.phoenix6.controls.DifferentialVelocityDutyCycle;
import com.ctre.phoenix6.controls.DifferentialVelocityVoltage;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicDutyCycle;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Position;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicDutyCycle_Position;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicDutyCycle_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicTorqueCurrentFOC_Position;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicTorqueCurrentFOC_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicVoltage_Position;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicVoltage_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_PositionDutyCycle_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionDutyCycle_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_PositionTorqueCurrentFOC_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionTorqueCurrentFOC_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_TorqueCurrentFOC_Position;
import com.ctre.phoenix6.controls.compound.Diff_TorqueCurrentFOC_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_VelocityDutyCycle_Position;
import com.ctre.phoenix6.controls.compound.Diff_VelocityDutyCycle_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_VelocityTorqueCurrentFOC_Position;
import com.ctre.phoenix6.controls.compound.Diff_VelocityTorqueCurrentFOC_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_VelocityVoltage_Position;
import com.ctre.phoenix6.controls.compound.Diff_VelocityVoltage_Velocity;
import com.ctre.phoenix6.controls.compound.Diff_VoltageOut_Position;
import com.ctre.phoenix6.controls.compound.Diff_VoltageOut_Velocity;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;


/** TalonFX */
public class LoggedTalonFX extends LoggableHardware {

  /**
   * TalonFX sensor inputs
   */
  @AutoLog
  public static class TalonFXInputs {
    public double selectedSensorPosition = 0.0;
    public double selectedSensorVelocity = 0.0;
  }

  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String CURRENT_LOG_ENTRY = "/Current";

  private TalonFX m_talon;
  private TalonFXInputsAutoLogged m_inputs;
  private String m_motorName;
  private double m_timeoutSeconds;

  private TalonFXConfiguration m_TalonFXConfiguration;

  //Feedback sensor types
  public enum FeedbackSensor {FUSED, REMOTE, SYNC}

  /**
   * Create a TalonFX object with built-in logging
   * @param id TalonFX ID
   */
  public LoggedTalonFX(int deviceID, String busName, String motorName) {
    this.m_talon = new TalonFX(deviceID, busName);
    this.m_inputs = new TalonFXInputsAutoLogged();
    this.m_motorName = motorName;
    this.m_timeoutSeconds = 0.050;

    PurpleManager.add(this);

    updateInputs();
    periodic();
  }

  /**
   * Create a TalonFX object with built-in logging
   * @param id TalonFX ID
   */
  public LoggedTalonFX(int deviceID, String busName, String motorName, double timeoutSeconds) {
    this.m_talon = new TalonFX(deviceID, busName);
    this.m_inputs = new TalonFXInputsAutoLogged();
    this.m_motorName = motorName;
    this.m_timeoutSeconds = timeoutSeconds;

    PurpleManager.add(this);

    updateInputs();
    periodic();
  }

  /**
   * Log output values
   * @param value Value that was set
   * @param mode The output mode to apply
   */
  private void logOutputs(ControlRequest mode, double value) {
    Logger.recordOutput(m_motorName + VALUE_LOG_ENTRY, value);
    Logger.recordOutput(m_motorName + MODE_LOG_ENTRY, mode.toString());
  }

  /**
   * Get the selected sensor position (in raw sensor units).
   *
   * @return Position of selected sensor (in raw sensor units).
   */
  public StatusSignal<Double> getSelectedSensorPosition() {
    return m_talon.getPosition().refresh();
  }

  /**
   * Get the selected sensor velocity.
   *
   * @return selected sensor (in raw sensor units) per 100ms.
   * See Phoenix-Documentation for how to interpret.
   */
  public StatusSignal<Double> getSelectedSensorVelocity() {
    return m_talon.getVelocity().refresh();
  }
 
  private void updateInputs() {
    synchronized (m_inputs) {
    m_inputs.selectedSensorPosition = getSelectedSensorPosition().getValue();
    m_inputs.selectedSensorVelocity = getSelectedSensorVelocity().getValue();
    }
  }

  @Override
  protected void periodic() {
    updateInputs();
    Logger.processInputs(m_motorName, m_inputs);
  }

  /**
   * Get latest sensor input data
   * @return Latest sensor data
   */
  @Override
  public TalonFXInputsAutoLogged getInputs() {
    return m_inputs;
  }

  public void setPosition(double position) {
    Logger.recordOutput(m_motorName + position + "",  position);
    m_talon.setPosition(position);
  }

  public void optimizeBusUtilization(double hz) {
    m_talon.optimizeBusUtilization(hz);
  }

  public void factoryDefaultConfig() {
    m_TalonFXConfiguration = new TalonFXConfiguration();
  }

  /**
   * Applies the contents of the specified config to the device.
   * <p>
   * This will wait up to {@link #DefaultTimeoutSeconds}.
   * <p>
   * Call to apply the selected configs.
   *
   * @param configs Configs to apply against.
   * @return StatusCode of the set command
   */
  public StatusCode applyConfigs(TalonFXConfiguration configs, double timeoutSeconds) {
     return m_talon.getConfigurator().apply(configs, timeoutSeconds);
  }

  public StatusCode applyConfigs(TalonFXConfiguration configs) {
    return applyConfigs(configs, 0.050);
  }

  /**
   * refreshs the configs
   * @param configs the configs to refresh
   */
  public StatusCode refreshConfigs(TalonFXConfiguration configs, double timeoutSeconds) {
    return m_talon.getConfigurator().apply(configs, timeoutSeconds);
  }

  public StatusCode refreshConfigs(TalonFXConfiguration configs) {
    return refreshConfigs(configs, 0.050);
  }

  public void setMotorConfigTimeout(double timeoutSeconds) {
    m_timeoutSeconds = timeoutSeconds;
  }

  public double getMotorConfigTimeout() {
    return m_timeoutSeconds;
  }

  /**
   * Initialize stator current limits
   * @param statorCurrentLimit 
   * The amount of current allowed in the motor (motoring and regen
   * current).  This is only applicable for non-torque current control
   * modes.  Note this requires the corresponding enable to be true.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0.0
   *   <li> <b>Maximum Value:</b> 800.0
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> A
   *   </ul>
   */
  public void initializeStatorCurrentLimits(TalonFXConfiguration config, double statorCurrentLimit, boolean statorCurrentEnable) {
    refreshConfigs(config, getMotorConfigTimeout());
      config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
      config.CurrentLimits.StatorCurrentLimitEnable = statorCurrentEnable;

    applyConfigs(config, getMotorConfigTimeout());
  }

  /**
   * Initialize supply current limits
   * @param supplyCurrentLimit
   * The amount of supply current allowed.  This is only applicable for
   * non-torque current control modes.  Note this requires the
   * corresponding enable to be true.  Use SupplyCurrentThreshold and
   * SupplyTimeThreshold to allow brief periods of high-current before
   * limiting occurs.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0.0
   *   <li> <b>Maximum Value:</b> 800.0
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> A
   *   </ul>
   */
  public void initializeSupplyCurrentLimits(TalonFXConfiguration config, double supplyCurrentLimit, boolean supplyCurrentEnable) {
    refreshConfigs(config, getMotorConfigTimeout());
      config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
      config.CurrentLimits.SupplyCurrentLimitEnable = supplyCurrentEnable;

   applyConfigs(config, getMotorConfigTimeout());
  }

  /**
   * Initialize supply current threshold
   * @param supplyCurrentThreshold
   * Delay supply current limiting until current exceeds this threshold
   * for longer than SupplyTimeThreshold.  This allows current draws
   * above SupplyCurrentLimit for a fixed period of time.  This has no
   * effect if SupplyCurrentLimit is greater than this value.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0.0
   *   <li> <b>Maximum Value:</b> 511
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> A
   *   </ul>
   */
  public void initializeSupplyCurrentThreshold(TalonFXConfiguration config, double supplyCurrentThreshold) {
    refreshConfigs(config, getMotorConfigTimeout());
      config.CurrentLimits.SupplyCurrentThreshold = supplyCurrentThreshold;

    applyConfigs(config, getMotorConfigTimeout());
  }

  /**
   * Initialize supply time threshold
   * 
   * Allows unlimited current for a period of time before current
   * limiting occurs.  Current threshold is the maximum of
   * SupplyCurrentThreshold and SupplyCurrentLimit.
   * 
   *   <ul>
   *   <li> <b>Minimum Value:</b> 0.0
   *   <li> <b>Maximum Value:</b> 1.275
   *   <li> <b>Default Value:</b> 0
   *   <li> <b>Units:</b> sec
   *   </ul>
   */
  public void initializeSupplyTimeThreshold(TalonFXConfiguration config, double supplyTimeThreshold) {
    refreshConfigs(config, getMotorConfigTimeout());
    config.CurrentLimits.SupplyTimeThreshold = supplyTimeThreshold;
   
    applyConfigs(config, getMotorConfigTimeout());
  }

  /**
   * Initiialize remote limit switches with TalonFX as the sensor
   * @param forward Boolean to choose whether to init forward limit switch
   * @param reverse Boolean to choose whether to init reverse limit switch
   */
  public void initializeRemoteLimitSwitches(TalonFXConfiguration config, boolean forward, boolean reverse) {
   refreshConfigs(config, getMotorConfigTimeout());
   if (forward) {
     config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = m_talon.getDeviceID();
   }
   if (reverse) {
     config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = m_talon.getDeviceID();
   }
   applyConfigs(config, getMotorConfigTimeout());
  }
  
  /**
   * Initialize remote limit switches with CANcoder as the sensor
   * <p> 
   * Take note of the fact that we are using our own version of CANCoder
   * @param cancoder Sensor for limit switches
   * @param forward Boolean to choose whether to init forward limit switch
   * @param reverse Boolean to choose whether to init reverse limit switch
   */
  public void initializeRemoteLimitSwitches(TalonFXConfiguration config, LoggedCANCoder cancoder, boolean forward, boolean reverse) {
   refreshConfigs(config, getMotorConfigTimeout());
   if (forward) {
    config.HardwareLimitSwitch.ForwardLimitRemoteSensorID = cancoder.getDeviceID();  
   }
   if (reverse) {
    config.HardwareLimitSwitch.ReverseLimitRemoteSensorID = cancoder.getDeviceID();
   }
   applyConfigs(config, getMotorConfigTimeout());
  }

  /**
   Choose what sensor source is reported via API and used by
   * closed-loop and limit features.  The default is RotorSensor, which
   * uses the internal rotor sensor in the Talon FX.  Choose
   * RemoteCANcoder to use another CANcoder on the same CAN bus (this
   * also requires setting FeedbackRemoteSensorID).  Talon FX will
   * update its position and velocity whenever CANcoder publishes its
   * information on CAN bus.  Choose FusedCANcoder (requires Phoenix
   * Pro) and Talon FX will fuse another CANcoder's information with the
   * internal rotor, which provides the best possible position and
   * velocity for accuracy and bandwidth (note this requires setting
   * FeedbackRemoteSensorID).  FusedCANcoder was developed for
   * applications such as swerve-azimuth.  Choose SyncCANcoder (requires
   * Phoenix Pro) and Talon FX will synchronize its internal rotor
   * position against another CANcoder, then continue to use the rotor
   * sensor for closed loop control (note this requires setting
   * FeedbackRemoteSensorID).  The TalonFX will report if its internal
   * position differs significantly from the reported CANcoder position.
   *  SyncCANcoder was developed for mechanisms where there is a risk of
   * the CANcoder failing in such a way that it reports a position that
   * does not match the mechanism, such as the sensor mounting assembly
   * breaking off.  Choose RemotePigeon2_Yaw, RemotePigeon2_Pitch, and
   * RemotePigeon2_Roll to use another Pigeon2 on the same CAN bus (this
   * also requires setting FeedbackRemoteSensorID).  Talon FX will
   * update its position to match the selected value whenever Pigeon2
   * publishes its information on CAN bus. Note that the Talon FX
   * position will be in rotations and not degrees.
   * <p>
   * Note: When the Talon Source is changed to FusedCANcoder, the Talon
   * needs a period of time to fuse before sensor-based (soft-limit,
   * closed loop, etc.) features are used. This period of time is
   * determined by the update frequency of the CANcoder's Position
   * signal.
   * @param cancoder CANCoder object to use for feedback
   * @param sensor Enum to choose which type of CANCoder
   */
  public void initializeFeedbackSensor(TalonFXConfiguration config, LoggedCANCoder cancoder, FeedbackSensor sensor) {
    refreshConfigs(config, getMotorConfigTimeout());

    //Automatically configure feedback sensor to built in rotor sensor
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    switch (sensor) {
      case REMOTE:
       config.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
       config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      case FUSED:
       config.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
       config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      case SYNC:
       config.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
       config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
      }
      applyConfigs(config, getMotorConfigTimeout());
    }

  /**
   * Initialize Talon PID Configuration and Motion Magic
   * @param pidconfig PID Config to use
   */
  public void initializeStaticFeedforwardSign(TalonFXConfiguration config, StaticFeedforwardSignValue staticFFSignValue) {
    refreshConfigs(config, getMotorConfigTimeout());

     /**
     * Static Feedforward Sign during position closed loop
     * <p>
     * This determines the sign of the applied kS during position
     * closed-loop modes. The default behavior uses the velocity
     * feedforward sign. This works well with position closed loop when
     * velocity reference is specified (motion profiling). However, when
     * using position closed loop with zero velocity reference (no motion
     * profiling), the application may want to apply static feedforward
     * based on the closed loop error sign instead. In which case, we
     * recommend the minimal amount of kS, otherwise the motor output may
     * dither when closed loop error is near zero.
     * 
     */
    config.Slot0.StaticFeedforwardSign = staticFFSignValue;

    applyConfigs(config, getMotorConfigTimeout());
   }

   /**
   * Initialize Talon PID Configuration and Motion Magic
   * @param pidconfig PID Config to use
   */
  public void initializeTalonPID(TalonFXConfiguration config, double kP, double kI, double kD) {
    refreshConfigs(config, getMotorConfigTimeout());

    //Configure PID Values
      config.Slot0.kP = kP;
      config.Slot0.kI = kI;
      config.Slot0.kD = kD;

    applyConfigs(config, getMotorConfigTimeout());
    
   }

   public void initializeFeedforward(TalonFXConfiguration config, double kS, double kV, double kA) {
    refreshConfigs(config, getMotorConfigTimeout());
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    applyConfigs(config, getMotorConfigTimeout());
   }

   public void initializeGravityFeedforward(TalonFXConfiguration config, double kG, GravityTypeValue gravityType) {
    refreshConfigs(config, getMotorConfigTimeout());
    config.Slot0.kG = kG;
    config.Slot0.GravityType = gravityType;
    applyConfigs(config, getMotorConfigTimeout());
   }

   /**
    * initizalizes Motion magic profile
    * @param velocity velocity constraint of the profile in units of rotations per second
    * @param acceleration acceleration constraint of the profile in units of rotations per second per second
    * @param jerk jerk constraint of the profile in units of rotations per second per second per second
    */
  public void initializeMotionMagic(TalonFXConfiguration config, double velocity, double acceleration, double jerk) {

    refreshConfigs(config, getMotorConfigTimeout());

     config.MotionMagic.MotionMagicCruiseVelocity = velocity;
     
     config.MotionMagic.MotionMagicAcceleration = acceleration;
     
     config.MotionMagic.MotionMagicJerk = jerk;

     applyConfigs(config, getMotorConfigTimeout());
  }

     /**
    * initizalizes Motion magic profile
    * @param velocity velocity constraint of the profile in units of rotations per second
    * @param acceleration acceleration constraint of the profile in units of rotations per second per second
    */
  public void initializeMotionMagic(TalonFXConfiguration config, double velocity, double acceleration) {
    initializeMotionMagic(config, velocity, acceleration);
  }

  public void initializeMotionMagicExpo(TalonFXConfiguration config, double expokV, double expokA) {
    refreshConfigs(config, getMotorConfigTimeout());

    config.MotionMagic.MotionMagicExpo_kV = expokV;

    config.MotionMagic.MotionMagicExpo_kA = expokA;

    applyConfigs(config, getMotorConfigTimeout());
  }

  public void setNeutralMode(TalonFXConfiguration config, NeutralModeValue neutralMode) {
    refreshConfigs(config, getMotorConfigTimeout());
    config.MotorOutput.NeutralMode = neutralMode;
    applyConfigs(config, getMotorConfigTimeout());
  }

  public void setInvert(TalonFXConfiguration config, InvertedValue invertDirection) {
    refreshConfigs(config, getMotorConfigTimeout());
    config.MotorOutput.Inverted = invertDirection;
    applyConfigs(config, getMotorConfigTimeout());
  }

   /**
     * Control motor with generic control request object.
     * <p>
     * User must make sure the specified object is castable to a valid control request,
     * otherwise this function will fail at run-time and return the NotSupported StatusCode
     *
     * @param request                Control object to request of the device
     * @return Status Code of the request, 0 is OK
     */
    public StatusCode setControl(ControlRequest request) {
      return m_talon.setControl(request);
    }


  public StatusCode setControl(VelocityVoltage request) {
    logOutputs(request, request.Velocity);
    return setControl(request);
  }

  public StatusCode setControl(PositionVoltage request) {
    logOutputs(request, request.Position);
    return setControl(request);
  }

  public StatusCode setControl(MotionMagicVoltage request) {
    logOutputs(request, request.Position);
    return setControl(request);
  }

  public StatusCode setControl(DutyCycleOut request) {
    logOutputs(request, request.Output);
    return setControl(request);
  }

  public StatusCode setControl(MotionMagicVelocityVoltage request) {
    logOutputs(request, request.Velocity);
    return setControl(request);
  }

  public StatusCode setControl(VoltageOut request) {
    logOutputs(request, request.Output);
    return setControl(request);
  }

  /**
  * Closes the TalonFX motor controller
  */
  @Override
  public void close() {
    PurpleManager.remove(this);
    m_talon.close();
  }
}
