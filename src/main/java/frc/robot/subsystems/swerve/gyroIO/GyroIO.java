package frc.robot.subsystems.swerve.gyroIO;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double pitch = 0.0;
        public double roll = 0.0;
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
        public double[] accel = new double[] {0, 0, 0};
        public double yawVelocity = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs){}
    
    public default void reset(){}

    public default void addOffset(Rotation2d offset) {}

}