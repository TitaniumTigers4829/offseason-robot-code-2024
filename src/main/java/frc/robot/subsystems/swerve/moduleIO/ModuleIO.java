package frc.robot.subsystems.swerve.moduleIO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    class ModuleIOInputs {
        public double driveWheelFinalRevolutions = 0.0;
        public double driveWheelFinalVelocityRevolutionsPerSec = 0.0;
        public double driveMotorAppliedVolts = 0.0;
        public double driveMotorCurrentAmps = 0;

        public Rotation2d turnRotation = new Rotation2d();
        public double steerVelocityRadPerSec = 0.0;
        public double steerMotorAppliedVolts = 0.0;
        public double steerMotorCurrentAmps = 0.0;

        public double[] odometryDriveWheelRevolutions = new double[]{};
        public Rotation2d[] odometrySteerPositions = new Rotation2d[]{};

        public boolean hardwareConnected = false;
        public Object drivePositionRad;
        public double driveVelocityRadPerSec;
        public double driveAppliedVolts;
        public double[] driveCurrentAmps;
        public Rotation2d turnAbsolutePosition;
        public Rotation2d turnPosition;
        public double turnVelocityRadPerSec;
        public double turnAppliedVolts;
        public double[] turnCurrentAmps;
        public double[] odometryTimestamps;
        public double[] odometryDrivePositionsRad;
        public Rotation2d[] odometryTurnPositions;
    }

    /**
     * Updates the inputs
     */
    void updateInputs(ModuleIOInputs inputs);

    default String getCANBus() {
        return "";
    };

    default void calibrate() {}

    default void setDesiredState(SwerveModuleState desiredState) {}

    /**
     * Run the drive motor at the specified percent speed.
     * @param speedPercent from -1 to 1, where 1 is the forward direction of the wheel
     */
    default void setDriveSpeed(double speedPercent) {}

    /**
     * Run the turn motor at the specified percent power.
     * @param powerPercent from -1 to 1, where 1 is counter-clockwise
     */
    default void setTurnSpeed(double powerPercent) {}

    /**
     * Enable or disable brake mode on the drive motor.
     */
    default void setDriveBrake(boolean enable) {}

    /**
     * Enable or disable brake mode on the turn motor.
     */
    default void setTurnBrake(boolean enable) {}

    default void setDriveVoltage(double voltage) {}

    default void setTurnVoltage(double voltage) {}

    default void stopModule() {}

    default double getTurnRotations() {
        return 0.0;
    }
}