// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.extras.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LogPaths;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Iron Maple's subsystem management. Based on {@link SubsystemBase} from WPILib, we added
 * on-disable/on-enable function calls as well as precise dt calculations
 */
public abstract class VirtualSubsystem extends SubsystemBase {
  public static final List<VirtualSubsystem> instances = new ArrayList<>();
  private double previousUpdateTimeStamp = 0;

  public static void register(VirtualSubsystem instance) {
    instances.add(instance);
  }

  public static void cancelRegister(VirtualSubsystem instance) {
    instances.remove(instance);
  }

  private static boolean wasEnabled = false;

  public static void checkForOnDisableAndEnable() {
    // periodic() is called from CommandScheduler, we only need to check for enable/disable
    if (DriverStation.isEnabled() && (!wasEnabled)) enableAlllSubsystems();
    else if (DriverStation.isDisabled() && (wasEnabled)) disableAllSubsystems();
    wasEnabled = DriverStation.isEnabled();
  }

  public static void enableAlllSubsystems() {
    for (VirtualSubsystem instance : instances) instance.onEnable();
  }

  public static void disableAllSubsystems() {
    for (VirtualSubsystem instance : instances) instance.onDisable();
  }

  public VirtualSubsystem(String name) {
    super(name);
    register(this);
  }

  public void onEnable() {}

  public void onDisable() {}

  public abstract void periodic(double dt, boolean enabled);

  @Override
  public void periodic() {
    final long t0 = System.nanoTime();
    periodic(getDt(), DriverStation.isEnabled());
    final double cpuTimeMS = (System.nanoTime() - t0) / 1_000_000.0;
    Logger.recordOutput(LogPaths.SYSTEM_PERFORMANCE_PATH + getName() + "-CPUTimeMS", cpuTimeMS);
  }

  private double getDt() {
    if (previousUpdateTimeStamp == 0) {
      previousUpdateTimeStamp = TimeUtil.getLogTimeSeconds();
      return Robot.defaultPeriodSecs;
    }
    final double dt = TimeUtil.getLogTimeSeconds() - previousUpdateTimeStamp;
    previousUpdateTimeStamp = Logger.getTimestamp();
    return dt;
  }
}
