// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveConstants.DriveTrainConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.extras.Alert;
import frc.robot.extras.VirtualSubsystem;
import frc.robot.subsystems.swerve.moduleIO.ModuleIO;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends VirtualSubsystem {
    private final ModuleIO io;
    private final String name;
    private final ModuleIO

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

        io.setDriveBrake(true);
        io.setTurnBrake(true);
    }

    public void updateOdometryInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module-" + name, inputs);
        this.hardwareFaultAlert.setActivated(!inputs.hardwareConnected);
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        updateOdometryPositions();
    }

    private void updateOdometryPositions() {
        odometryPositions = new SwerveModulePosition[inputs.odometryDriveWheelRevolutions.length];
        for (int i = 0; i < odometryPositions.length; i++) {
            double positionMeters = driveWheelRevolutionsToMeters(inputs.odometryDriveWheelRevolutions[i]);
            Rotation2d angle = inputs.odometrySteerPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        Logger.recordOutput("Drive/Module-" + name + " desiredState", desiredState);
        io.setDesiredState(desiredState);
    }

    @Override
    public void onDisable() {
        io.setTurnSpeed(0);
        io.setDriveSpeed(0);
    }

    public void stopModule () {
        io.stopModule();
    }

    /**
     * Returns the current turn angle of the module.
     */
    public Rotation2d getTurnRotation() {
        return inputs.turnRotation;
    }

    public double getSteerVelocityRadPerSec() {
        return inputs.steerVelocityRadPerSec;
    }

    /**
     * Returns the current drive position of the module in meters.
     */
    public double getDrivePositionMeters() {
        return driveWheelRevolutionsToMeters(inputs.driveWheelFinalRevolutions);
    }

    private double driveWheelRevolutionsToMeters(double driveWheelRevolutions) {
        return Units.rotationsToRadians(driveWheelRevolutions) * DriveTrainConstants.WHEEL_RADIUS_METERS;
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getDriveVelocity() {
        return driveWheelRevolutionsToMeters(inputs.driveWheelFinalVelocityRevolutionsPerSec);
    }

    /**
     * Returns the module position (turn angle and drive position).
     */
    public SwerveModulePosition getLatestPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getTurnRotation());
    }

    /**
     * Returns the module state (turn angle and drive velocity).
     */
    public SwerveModuleState getMeasuredState() {
        return new SwerveModuleState(getDriveVelocity(), getTurnRotation());
    }

    /**
     * Returns the module positions received this cycle.
     */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }
}