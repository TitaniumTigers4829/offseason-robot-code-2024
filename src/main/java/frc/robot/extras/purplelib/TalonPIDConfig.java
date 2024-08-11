// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.


package frc.robot.extras.purplelib;


import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/**
 * Automates the configuration of Talon PID (v6) and MotionMagic parameters
 */
public class TalonPIDConfig {
  private static final double MIN_TOLERANCE = 1.0;
  private static final double MIN_JERK = 0;
  private static final double MAX_JERK = 9999;


  private boolean m_motionMagic = false;
  private boolean m_enableSoftLimits = true;


  private boolean m_sensorPhase = false;
  private boolean m_invertMotor = false;
  private double m_ticksPerRotation = 0.0;
  private double m_kP = 0.0;
  private double m_kI = 0.0;
  private double m_kD = 0.0;
  private double m_kF = 0.0;
  private double m_kS = 0.0;
  private double m_kG = 0.0;
  private double m_kV = 0.0;
  private double m_kA = 0.0;
  private double m_tolerance = 1.0;
  private double m_lowerLimit = 0.0;
  private double m_upperLimit = 0.0;


  private double m_velocityRPS = 1.0;
  private double m_accelerationRPS = 1.0;
  private double m_jerk = 0;
  private GravityTypeValue m_gravitytype;
  private StaticFeedforwardSignValue m_staticff;

public TalonPIDConfig(double P, double I, double D,
 double S, double V, double A, boolean motionMagic,
 double velocity, double acceleration, double jerk, GravityTypeValue gravitytype, StaticFeedforwardSignValue staticff, double G) {
  this.m_kP = P;
  this.m_kI = I;
  this.m_kD = D;
  this.m_kS = S;
  this.m_kV = V;
  this.m_kA = A;
  this.m_velocityRPS = velocity;
  this.m_accelerationRPS = acceleration;
  this.m_gravitytype = gravitytype;
  this.m_kG = G;
  this.m_staticff = staticff;
  this.m_motionMagic = motionMagic;
 }

  /**
   * @return Proportional gain
   */
  public double getkP() {
    return m_kP;
  }


  /**
   * @return Integral gain
   */
  public double getkI() {
    return m_kI;
  }


  /**
   * @return Derivative gain
   */
  public double getkD() {
    return m_kD;
  }


  /**
   * @return Feed-forward gain
   */
  public double getkF() {
    return m_kF;
  }


  /**
   * @return Static gain
   */
  public double getkS() {
    return m_kS;
  }


  /**
   * @return Gravity gain
   */
  public double getkG() {
    return m_kG;
  }


  /**
   * @return Velocity gain
   */
  public double getkV() {
    return m_kV;
  }


  /**
   * @return Acceleration gain
   */
  public double getkA() {
    return m_kA;
  }

  /**
   * @return MotionMagic cruise velocity in RPM
   */
  public double getVelocityRPS() {
    return m_velocityRPS;
  }


  /**
   * @return MotionMagic acceleration in RPM per sec
   */
  public double getAccelerationRPS() {
    return m_accelerationRPS;
  }


  /**
   * @return Whether motion magic is enabled or not
   */
  public boolean getMotionMagic() {
    return m_motionMagic;
  }


  /**
   * @return MotionMagic jerk
   */
  public double getMotionMagicJerk() {
    return m_jerk;
  }

  /**
   * @return Gravity type
   */
  public GravityTypeValue getGravityType() {
    return m_gravitytype;
  }

  /**
   * @return Static feedforward
   */
  public StaticFeedforwardSignValue getStaticFeedForward() {
    return m_staticff;
  }
}
