package frc.robot.subsystems.swerve.odometryThread;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.Robot;
import frc.robot.extras.util.DeviceCANBus;
import frc.robot.extras.util.TimeUtil;
import frc.robot.subsystems.swerve.SwerveConstants.DriveTrainConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

public interface OdometryThread {
  final class OdometryDoubleInput {
    private final Supplier<Double> supplier;
    private final Queue<Double> queue;

    public OdometryDoubleInput(Supplier<Double> signal) {
      this.supplier = signal;
      this.queue = new ArrayBlockingQueue<>(DriveTrainConstants.ODOMETRY_CACHE_CAPACITY);
    }

    public void cacheInputToQueue() {
      this.queue.offer(supplier.get());
    }
  }

  List<OdometryDoubleInput> registeredInputs = new ArrayList<>();
  List<BaseStatusSignal> registeredStatusSignals = new ArrayList<>();

  static Queue<Double> registerSignalInput(StatusSignal<Double> signal) {
    signal.setUpdateFrequency(
        DriveTrainConstants.ODOMETRY_FREQUENCY, DriveTrainConstants.ODOMETRY_WAIT_TIMEOUT_SECONDS);
    registeredStatusSignals.add(signal);
    return registerInput(signal.asSupplier());
  }

  static Queue<Double> registerInput(Supplier<Double> supplier) {
    final OdometryDoubleInput odometryDoubleInput = new OdometryDoubleInput(supplier);
    registeredInputs.add(odometryDoubleInput);
    return odometryDoubleInput.queue;
  }

  static OdometryThread createInstance(DeviceCANBus canBus) {
    return switch (Robot.CURRENT_ROBOT_MODE) {
      case REAL ->
          new OdometryThreadReal(
              canBus,
              registeredInputs.toArray(new OdometryDoubleInput[0]),
              registeredStatusSignals.toArray(new BaseStatusSignal[0]));
      case SIM -> new OdometryThreadSim();
      case REPLAY -> inputs -> {};
    };
  }

  @AutoLog
  class OdometryThreadInputs {
    public double[] measurementTimeStamps = new double[0];
  }

  void updateInputs(OdometryThreadInputs inputs);

  default void start() {}

  default void lockOdometry() {}

  default void unlockOdometry() {}

  final class OdometryThreadSim implements OdometryThread {
    @Override
    public void updateInputs(OdometryThreadInputs inputs) {
      inputs.measurementTimeStamps = new double[DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD];
      final double robotStartingTimeStamps = TimeUtil.getLogTimeSeconds(),
          iterationPeriodSeconds =
              Robot.defaultPeriodSecs / DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD;
      for (int i = 0; i < DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD; i++)
        inputs.measurementTimeStamps[i] = robotStartingTimeStamps + i * iterationPeriodSeconds;
    }
  }
}
