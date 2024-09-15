package frc.robot.extras.simulation.fieldDisplay;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.*;

public abstract class GamePieceOnFlyDisplay implements CompetitionFieldVisualizer.ObjectOnFieldDisplay {
    private final Translation3d shooterPosition, targetedPosition;
    private final double flightTimeSeconds, startTimeSeconds;
    private final Rotation3d gamePieceRotation;

    public GamePieceOnFlyDisplay(Translation3d shooterPosition, Translation3d targetedPosition, double flightTimeSeconds) {
        this.shooterPosition = shooterPosition;
        this.targetedPosition = targetedPosition;
        this.flightTimeSeconds = flightTimeSeconds;
        this.startTimeSeconds = getLogTimeSeconds();

        final Translation3d displacementToTarget = targetedPosition.minus(shooterPosition);
        final double yaw = displacementToTarget.toTranslation2d().getAngle().getRadians(),
                pitch = -Math.atan2(displacementToTarget.getZ(), displacementToTarget.toTranslation2d().getNorm());
        this.gamePieceRotation = new Rotation3d(0, pitch, yaw);
    }

    @Override
    public Pose3d getPose3d() {
        return new Pose3d(shooterPosition.interpolate(targetedPosition, getT()), gamePieceRotation);
    }

    public boolean isReached() {
        return getT() >= 1;
    }

    public double getT() {
        return (getLogTimeSeconds() - startTimeSeconds) / flightTimeSeconds;
    }

    public static double getLogTimeSeconds() {
        return Logger.getTimestamp() / 1_000_000.0;
    }
}