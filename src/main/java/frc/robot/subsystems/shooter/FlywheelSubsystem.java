// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.Constants.FlywheelConstants;


public class FlywheelSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  // private final SimpleMotorFeedforward ffModel;
  private final SysIdRoutine sysId;

  // private final VelocityVoltage velocityRequest;
  // private final StatusSignal<Double> flywheelVelocity;
  
  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;
    io.configurePID(FlywheelConstants.FLYWHEEL_P, FlywheelConstants.FLYWHEEL_I, FlywheelConstants.FLYWHEEL_D);
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
            io.updateInputs(inputs);
    // velocityRequest = new VelocityVoltage(0);
    // TalonFXConfiguration FlywheelConfig = new TalonFXConfiguration();
    // FlywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // FlywheelConfig.Slot0.kP = 0.65;
    // FlywheelConfig.Slot0.kI = 0;
    // FlywheelConfig.Slot0.kD = 0;
    // FlywheelConfig.Slot0.kS = 0.25;
    // FlywheelConfig.Slot0.kV = 0;
    // FlywheelConfig.Slot0.kA = 0;

    // flywheelVelocity = leftFlywheel.getAcceleration();
    // flywheelVelocity.setUpdateFrequency(250);
    // leftFlywheel.getConfigurator().apply(FlywheelConfig);
    // FlywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // rightFlywheelMotor.getConfigurator().apply(FlywheelConfig);
  }

  // public void setFlywheelVoltage(double desiredVoltage) {
  //   leftFlywheel.setVoltage(desiredVoltage);
  //   rightFlywheelMotor.setVoltage(desiredVoltage);
  // }
  // public void spin(double speed) {
  //   leftFlywheel.set(speed);
  //   rightFlywheelMotor.set(speed);
  // }

  // public void setFlywheelVelocity(double desiredRPM) {
  //   leftFlywheel.setControl(velocityRequest.withVelocity(desiredRPM / 60.0));
  //   rightFlywheelMotor.setControl(velocityRequest.withVelocity(desiredRPM / 60.0));
  // }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // flywheelVelocity.refresh();
    // SmartDashboard.putNumber("Acceleration actual", flywheelVelocity.getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
