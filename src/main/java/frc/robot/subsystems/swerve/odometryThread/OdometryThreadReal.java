// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/
package frc.robot.subsystems.swerve.odometryThread;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.extras.CANTHINGY.DeviceCANBus;
import frc.robot.extras.TimeUtils;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread.OdometryDoubleInput;
import frc.robot.subsystems.swerve.odometryThread.OdometryThread.OdometryThreadInputs;

import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class OdometryThreadReal extends Thread implements OdometryThread {
    private final DeviceCANBus canBus;
    private final OdometryDoubleInput[] odometryDoubleInputs;
    private final BaseStatusSignal[] statusSignals;
    private final Queue<Double> timeStampsQueue;
    private final Lock lock = new ReentrantLock();
    public OdometryThreadReal(DeviceCANBus canBus, OdometryDoubleInput[] odometryDoubleInputs, BaseStatusSignal[] statusSignals) {
        this.timeStampsQueue = new ArrayBlockingQueue<>(DriveTrainConstants.ODOMETRY_CACHE_CAPACITY);
        this.canBus = canBus;
        this.odometryDoubleInputs = odometryDoubleInputs;
        this.statusSignals = statusSignals;

        setName("OdometryThread");
        setDaemon(true);
    }

    @Override
    public synchronized void start() {
        if (odometryDoubleInputs.length > 0)
            super.start();
    }


    @Override
    public void run() {
        while (true) odometryPeriodic();
    }

    private void odometryPeriodic() {
        refreshSignalsAndBlockThread();

        lock.lock();
        timeStampsQueue.offer(estimateAverageTimeStamps());
        for(OdometryDoubleInput odometryDoubleInput : odometryDoubleInputs)
            odometryDoubleInput.cacheInputToQueue();
        lock.unlock();
    }

    private void refreshSignalsAndBlockThread() {
        switch (canBus) {
            case RIO -> {
                TimeUtils.delay(1.0 / DriveTrainConstants.ODOMETRY_FREQUENCY);
                BaseStatusSignal.refreshAll();
            }
            case CANIVORE ->
                    BaseStatusSignal.waitForAll(DriveTrainConstants.ODOMETRY_WAIT_TIMEOUT_SECONDS, statusSignals);
        }
    }

    private double estimateAverageTimeStamps() {
        double currentTime = TimeUtils.getRealTimeSeconds(), totalLatency = 0;
        for (BaseStatusSignal signal:statusSignals)
            totalLatency += signal.getTimestamp().getLatency();

        if (statusSignals.length == 0)
            return currentTime;
        return currentTime - totalLatency / statusSignals.length;
    }


    @Override
    public void updateInputs(OdometryThreadInputs inputs) {
        inputs.measurementTimeStamps = new double[timeStampsQueue.size()];
        for (int i = 0; i < inputs.measurementTimeStamps.length && !timeStampsQueue.isEmpty(); i++)
            inputs.measurementTimeStamps[i] = timeStampsQueue.poll();
    }

    @Override
    public void lockOdometry() {
        lock.lock();
    }

    @Override
    public void unlockOdometry() {
        lock.unlock();
    }
}