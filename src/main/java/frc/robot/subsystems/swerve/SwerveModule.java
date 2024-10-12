// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.Alert;
import frc.robot.extras.simulation.SwerveStateProjection;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConfig;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.moduleIO.ModuleIO;
import frc.robot.subsystems.swerve.moduleIO.ModuleIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;


public class SwerveModule extends SubsystemBase {
    private final ModuleIO io;
    private final String name;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    // private final PIDController turnCloseLoop, driveCloseLoop;
    private SwerveModuleState setPoint;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[]{};

    private final Alert hardwareFaultAlert;

    public SwerveModule(ModuleIO io, String name) {
        super("Module-" + name);
        this.io = io;
        this.name = name;
        this.hardwareFaultAlert = new Alert(
                "Module-" + name + " Hardware Fault",
                Alert.AlertType.ERROR
        );
        this.hardwareFaultAlert.setActivated(false);

        CommandScheduler.getInstance().unregisterSubsystem(this);

        setPoint = new SwerveModuleState();
        // turnCloseLoop.calculate(getSteerFacing().getRadians()); // activate close loop controller
        io.setDriveBrake(true);
    }

    public void updateOdometryInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module-" + name, inputs);
        this.hardwareFaultAlert.setActivated(!inputs.isConnected);
    }

    @Override
    public void periodic() {
        updateOdometryPositions();
    }

    private void updateOdometryPositions() {
        odometryPositions = new SwerveModulePosition[inputs.odometryDriveWheelRevolutions.length];
        for (int i = 0; i < odometryPositions.length; i++) {
            double positionMeters = driveWheelRevolutionsToMeters(inputs.odometryDriveWheelRevolutions[i]);
            Rotation2d angle = inputs.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

  
    /**
     * Runs the module with the specified setpoint state. Returns the optimized state.
     */
    public SwerveModuleState runSetPoint(SwerveModuleState desiredState) {
        this.setPoint = SwerveModuleState.optimize(desiredState, getSteerFacing());
      // final double adjustSpeedSetpointMetersPerSec = SwerveStateProjection.project(desiredState, Rotation2d.fromRotations(io.getTurnRotations()));


        io.setDesiredState(setPoint);

        return this.setPoint;
    }

    // @Override
    // public void onDisable() {
    //     // io.setSteerPowerPercent(0);
    //     io.setDriveVoltage(0);
    // }

    /**
     * Returns the current turn angle of the module.
     */
    public Rotation2d getSteerFacing() {
        return inputs.turnAbsolutePosition;
    }

    public double getSteerVelocityRadPerSec() {
        return inputs.steerVelocityRadPerSec;
    }

    /**
     * Returns the current drive position of the module in meters.
     */
    public double getDrivePositionMeters() {
        return driveWheelRevolutionsToMeters(inputs.drivePosition);
    }

    private double driveWheelRevolutionsToMeters(double driveWheelRevolutions) {
        return Units.rotationsToRadians(driveWheelRevolutions) * ModuleConstants.WHEEL_DIAMETER_METERS;
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getDriveVelocityMetersPerSec() {
        return driveWheelRevolutionsToMeters(inputs.driveVelocity);
    }

    /**
     * Returns the module position (turn angle and drive position).
     */
    public SwerveModulePosition getLatestPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getSteerFacing());
    }

    /**
     * Returns the module state (turn angle and drive velocity).
     */
    public SwerveModuleState getMeasuredState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSec(), getSteerFacing());
    }

    /**
     * Returns the module positions received this cycle.
     */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }
}
