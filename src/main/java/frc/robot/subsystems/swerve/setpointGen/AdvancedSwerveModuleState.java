package frc.robot.subsystems.swerve.setpointGen;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.struct.Struct;

public final class AdvancedSwerveModuleState extends SwerveModuleState {
  public double steerVelocityFF;
  public double driveAccelerationFF;

  public AdvancedSwerveModuleState(
      double speedMetersPerSecond,
      Rotation2d angle,
      double steerVelocityFF,
      double driveAccelerationFF) {
    super(speedMetersPerSecond, angle);
    this.steerVelocityFF = steerVelocityFF;
    this.driveAccelerationFF = driveAccelerationFF;
  }

  // todo: implement custom struct
  public static final Struct<SwerveModuleState> struct = SwerveModuleState.struct;

  public static AdvancedSwerveModuleState fromBase(SwerveModuleState base) {
    return new AdvancedSwerveModuleState(base.speedMetersPerSecond, base.angle, 0.0, 0.0);
  }

  // public static AdvancedSwerveModuleState[] fromBase(SwerveModuleState[] base) {
  //   AdvancedSwerveModuleState[] state = new AdvancedSwerveModuleState[base.length];
  //   for(int i = 0; i < base.length; i++) {
  //     state[i] = new AdvancedSwerveModuleState(base[i].speedMetersPerSecond, base[i].angle, 0.0,
  // 0.0);
  //   }
  //   return state;
  // }
}
