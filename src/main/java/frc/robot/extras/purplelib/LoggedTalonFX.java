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


/** TalonFX */
public class LoggedTalonFX extends LoggableHardware {

  /**
   * TalonFX sensor inputs
   */
  @AutoLog
  class TalonFXInputs {
    public double selectedSensorPosition = 0.0;
    public double selectedSensorVelocity = 0.0;
  }

  private static final String VALUE_LOG_ENTRY = "/OutputValue";
  private static final String MODE_LOG_ENTRY = "/OutputMode";
  private static final String CURRENT_LOG_ENTRY = "/Current";

  private TalonFX m_talon;
  private TalonFXInputsAutoLogged m_inputs;
  private String m_motorName;

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


    // Disable motor safety
    m_talon.setSafetyEnabled(false);

    PurpleManager.add(this);

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
    m_talon.getPosition().refresh();
    return m_talon.getPosition();
  }

  /**
   * Get the selected sensor velocity.
   *
   * @return selected sensor (in raw sensor units) per 100ms.
   * See Phoenix-Documentation for how to interpret.
   */
  public StatusSignal<Double> getSelectedSensorVelocity() {
    m_talon.getVelocity().refresh();
    return m_talon.getVelocity();
  }
 
  private void updateInputs() {
    m_inputs.selectedSensorPosition = getSelectedSensorPosition().getValue();
    m_inputs.selectedSensorVelocity = getSelectedSensorVelocity().getValue();
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
    return applyConfigs(configs);
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
  public void initializeStatorCurrentLimits(double statorCurrentLimit, boolean statorCurrentEnable, double timeoutSeconds) {
    factoryDefaultConfig();
    TalonFXConfiguration statorConfigs = m_TalonFXConfiguration;
      statorConfigs.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
      statorConfigs.CurrentLimits.StatorCurrentLimitEnable = statorCurrentEnable;

    applyConfigs(statorConfigs, timeoutSeconds);
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
  public void initializeSupplyCurrentLimits(double supplyCurrentLimit, boolean supplyCurrentEnable, double timeoutSeconds) {
    factoryDefaultConfig();
    TalonFXConfiguration supplyConfigs = m_TalonFXConfiguration;
      supplyConfigs.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
      supplyConfigs.CurrentLimits.SupplyCurrentLimitEnable = supplyCurrentEnable;

   applyConfigs(supplyConfigs, timeoutSeconds);
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
  public void initializeSupplyCurrentThreshold(double supplyCurrentThreshold) {
    CurrentLimitsConfigs limitConfigs = m_TalonFXConfiguration.CurrentLimits;
      limitConfigs.SupplyCurrentThreshold = supplyCurrentThreshold;

     m_talon.getConfigurator().apply(limitConfigs);
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
  public void initializeSupplyTimeThreshold(double supplyTimeThreshold) {
    CurrentLimitsConfigs limitConfigs = m_TalonFXConfiguration.CurrentLimits;
    limitConfigs.SupplyTimeThreshold = supplyTimeThreshold;
   
    m_talon.getConfigurator().apply(limitConfigs);
  }

  /**
   * Initiialize remote limit switches with TalonFX as the sensor
   * @param forward Boolean to choose whether to init forward limit switch
   * @param reverse Boolean to choose whether to init reverse limit switch
   */
  public void initializeRemoteLimitSwitches(boolean forward, boolean reverse) {
   HardwareLimitSwitchConfigs limitConfigs = m_TalonFXConfiguration.HardwareLimitSwitch;
   if (forward) {
     limitConfigs.ForwardLimitRemoteSensorID = m_talon.getDeviceID();
   }
   if (reverse) {
     limitConfigs.ReverseLimitRemoteSensorID = m_talon.getDeviceID();
   }
   m_talon.getConfigurator().apply(limitConfigs);
  }
  
  /**
   * Initialize remote limit switches with CANcoder as the sensor
   * <p> 
   * Take note of the fact that we are using our own version of CANCoder
   * @param cancoder Sensor for limit switches
   * @param forward Boolean to choose whether to init forward limit switch
   * @param reverse Boolean to choose whether to init reverse limit switch
   */
  public void initializeRemoteLimitSwitches(LoggedCANCoder cancoder, boolean forward, boolean reverse) {
   HardwareLimitSwitchConfigs limitConfigs = m_TalonFXConfiguration.HardwareLimitSwitch;
   if (forward) {
    limitConfigs.ForwardLimitRemoteSensorID = cancoder.getDeviceID();  
   }
   if (reverse) {
    limitConfigs.ReverseLimitRemoteSensorID = cancoder.getDeviceID();
   }
   m_talon.getConfigurator().apply(limitConfigs);
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
  public void initializeFeedbackSensor(LoggedCANCoder cancoder, FeedbackSensor sensor) {
    factoryDefaultConfig();
    FeedbackConfigs config = m_TalonFXConfiguration.Feedback;

    //Automatically configure feedback sensor to built in rotor sensor
    config.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    switch (sensor) {
      case REMOTE:
       config.FeedbackRemoteSensorID = cancoder.getDeviceID();
       config.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      case FUSED:
       config.FeedbackRemoteSensorID = cancoder.getDeviceID();
       config.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      case SYNC:
       config.FeedbackRemoteSensorID = cancoder.getDeviceID();
       config.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
      }
      m_talon.getConfigurator().apply(config);
    }

  /**
   * Initialize Talon PID Configuration and Motion Magic
   * @param pidconfig PID Config to use
   */
  public void initializeTalonPID(double kP, double kI, double kD) {
    factoryDefaultConfig();
    TalonFXConfiguration slot0Configs = m_TalonFXConfiguration;

    //Configure PID Values
      slot0Configs.Slot0.kP = kP;
      slot0Configs.Slot0.kI = kI;
      slot0Configs.Slot0.kD = kD;
      // slot0Configs.kS = kS;
      // slot0Configs.kV = kV;
      // slot0Configs.kA = kA;

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
  // slot0Configs.StaticFeedforwardSign = pidconfig.getStaticFeedForward();

    applyConfigs(slot0Configs);
    
   }

   public void initializeTalonFeedForward(double kS, double kV, double kA, double kG, GravityTypeValue gravityType) {
    factoryDefaultConfig();
    Slot0Configs slot0Configs = m_TalonFXConfiguration.Slot0;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;
    slot0Configs.kA = kA;
    slot0Configs.kG = kG;
    slot0Configs.GravityType = gravityType;
    m_talon.getConfigurator().apply(slot0Configs);
   }

   /**
    * initizalizes Motion magic profile
    * @param velocity velocity constraint of the profile in units of rotations per second
    * @param acceleration acceleration constraint of the profile in units of rotations per second per second
    * @param jerk jerk constraint of the profile in units of rotations per second per second per second
    */
  public void initializeMotionMagic(double velocity, double acceleration, double jerk) {
    factoryDefaultConfig();
    //Configure MotionMagic
    MotionMagicConfigs m_motionMagic = m_TalonFXConfiguration.MotionMagic;
     /**
     * This is the maximum velocity Motion Magic based control modes are
     * allowed to use.  Motion Magic Velocity control modes do not use
     * this config.  When using Motion Magic Expo control modes, setting
     * this to 0 will allow the profile to run to the max possible
     * velocity based on Expo_kV.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 9999
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rps
     *   </ul>
     */
     m_motionMagic.MotionMagicCruiseVelocity = velocity;
     /**
     * This is the target acceleration Motion Magic based control modes
     * are allowed to use.  Motion Magic Expo control modes do not use
     * this config.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 9999
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rot per sec^2
     *   </ul>
     */
     m_motionMagic.MotionMagicAcceleration = acceleration;
     
     /**
     * This is the target jerk (acceleration derivative) Motion Magic
     * based control modes are allowed to use.  Motion Magic Expo control
     * modes do not use this config.  This allows Motion Magic support of
     * S-Curves.  If this is set to zero, then Motion Magic will not
     * apply a Jerk limit.
     * 
     *   <ul>
     *   <li> <b>Minimum Value:</b> 0
     *   <li> <b>Maximum Value:</b> 9999
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> rot per sec^2
     *   </ul>
     */
     m_motionMagic.MotionMagicJerk = jerk;

     m_talon.getConfigurator().apply(m_motionMagic);
  }

  /**
    * initizalizes Motion magic profile
    * @param velocity velocity constraint of the profile in units of rotations per second
    * @param acceleration acceleration constraint of the profile in units of rotations per second per second
    */
  public void initializeMotionMagic(double velocity, double acceleration) {
    initializeMotionMagic(velocity, acceleration, 0);
  }

    /**
    * initizalizes Motion magic profile
    * @param velocity velocity constraint of the profile in units of rotations per second
    */
  public void initializeMotionMagic(double velocity) {
    initializeMotionMagic(velocity, 0, 0);
  }

  public void initializeMotionMagicExpo(double expokV, double expokA) {
    factoryDefaultConfig();
    MotionMagicConfigs m_motionMagicExpo = m_TalonFXConfiguration.MotionMagic;

    m_motionMagicExpo.MotionMagicExpo_kV = expokV;

    m_motionMagicExpo.MotionMagicExpo_kA = expokA;

    m_talon.getConfigurator().apply(m_motionMagicExpo);
  }

  public void setNeutralMode(NeutralModeValue neutralMode) {
    factoryDefaultConfig();
    TalonFXConfiguration neutralConfig = m_TalonFXConfiguration;
    neutralConfig.MotorOutput.NeutralMode = neutralMode;
    applyConfigs(neutralConfig);
  }

  public void setInvert(InvertedValue invertDirection) {
    factoryDefaultConfig();
    TalonFXConfiguration invertConfig = m_TalonFXConfiguration;
    invertConfig.MotorOutput.Inverted = invertDirection;
    applyConfigs(invertConfig);
  }

  /**
   * Request PID to target position with duty cycle feedforward.
   * <p>
   * This control mode will set the motor's position setpoint to the
   * position specified by the user. In addition, it will apply an
   * additional duty cycle as an arbitrary feedforward value.
   *   <ul>
   *   <li> <b>PositionDutyCycle Parameters:</b> 
   *    <ul>
   *    <li> <b>Position:</b> Position to drive toward in rotations.
   *    <li> <b>Velocity:</b> Velocity to drive toward in rotations per second. This
   *                       is typically used for motion profiles generated by the
   *                       robot program.
   *    <li> <b>EnableFOC:</b> Set to true to use FOC commutation (requires Phoenix
   *                        Pro), which increases peak power by ~15%. Set to false
   *                        to use trapezoidal commutation.  FOC improves motor
   *                        performance by leveraging torque (current) control. 
   *                        However, this may be inconvenient for applications
   *                        that require specifying duty cycle or voltage. 
   *                        CTR-Electronics has developed a hybrid method that
   *                        combines the performances gains of FOC while still
   *                        allowing applications to provide duty cycle or voltage
   *                        demand.  This not to be confused with simple
   *                        sinusoidal control or phase voltage control which
   *                        lacks the performance gains.
   *    <li> <b>FeedForward:</b> Feedforward to apply in fractional units between -1
   *                          and +1.
   *    <li> <b>Slot:</b> Select which gains are applied by selecting the slot.  Use
   *                   the configuration api to set the gain values for the
   *                   selected slot before enabling this feature. Slot must be
   *                   within [0,2].
   *    <li> <b>OverrideBrakeDurNeutral:</b> Set to true to static-brake the rotor
   *                                      when output is zero (or within
   *                                      deadband).  Set to false to use the
   *                                      NeutralMode configuration setting
   *                                      (default). This flag exists to provide
   *                                      the fundamental behavior of this control
   *                                      when output is zero, which is to provide
   *                                      0V to the motor.
   *    <li> <b>LimitForwardMotion:</b> Set to true to force forward limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    <li> <b>LimitReverseMotion:</b> Set to true to force reverse limiting.  This
   *                                 allows users to use other limit switch
   *                                 sensors connected to robot controller.  This
   *                                 also allows use of active sensors that
   *                                 require external power.
   *    </ul>
   *   </ul>
   *
   * @param request                Control object to request of the device
   * @return Code response of the request
   */
  public StatusCode setControl(ControlRequest request)
  {
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
