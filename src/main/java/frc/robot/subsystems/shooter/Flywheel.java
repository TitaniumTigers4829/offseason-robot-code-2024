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
import frc.robot.Constants.HardwareConstants;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;
  
  public Flywheel(FlywheelIO io) {
    this.io = io;
    switch (HardwareConstants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(FlywheelConstants.FLYWHEEL_P, FlywheelConstants.FLYWHEEL_I, FlywheelConstants.FLYWHEEL_D);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  public void setFlywheelVoltage(double desiredVoltage) {
    io.setVoltage(desiredVoltage);
  }

  public void setFlywheelVelocity(double desiredRPM, double ffVolts) {
    io.setVelocity(desiredRPM, ffVolts);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
