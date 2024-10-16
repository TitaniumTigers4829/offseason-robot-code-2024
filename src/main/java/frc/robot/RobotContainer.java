package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HardwareConstants;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.extras.characterization.FeedForwardCharacterization;
import frc.robot.extras.simulation.SimulatedField;
import frc.robot.extras.simulation.physicsSim.GyroSimulation;
import frc.robot.extras.simulation.physicsSim.SwerveDriveSimulation;
import frc.robot.extras.simulation.physicsSim.SwerveModuleSimulation;
import frc.robot.extras.simulation.physicsSim.SwerveModuleSimulation.DRIVE_WHEEL_TYPE;
import frc.robot.subsystems.swerve.SwerveConstants;
// import frc.robot.extras.characterization.WheelRadiusCharacterization;
// import frc.robot.extras.characterization.WheelRadiusCharacterization.Direction;
// import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.gyroIO.GyroIO;
import frc.robot.subsystems.swerve.gyroIO.GyroIONavX;
import frc.robot.subsystems.swerve.gyroIO.GyroIOSim;
import frc.robot.subsystems.swerve.moduleIO.ModuleIO;
import frc.robot.subsystems.swerve.moduleIO.ModuleIOSim;
import frc.robot.subsystems.swerve.moduleIO.ModuleIOTalonFX;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  // Simulation, we store them here in the robot container
  // private final SimulatedField simulatedArena;
  private final SwerveDriveSimulation swerveDriveSimulation;
  private final GyroSimulation gyroSimulation;

  // Subsystems
  private final SwerveDrive driveSubsystem;
  private final XboxController driverController = new XboxController(0);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        /* Real robot, instantiate hardware IO implementations */

        /* Disable Simulations */
        // this.simulatedArena = null;
        this.gyroSimulation = null;
        this.swerveDriveSimulation = null;

        driveSubsystem =
            new SwerveDrive(
                new GyroIONavX(),
                new ModuleIOTalonFX(SwerveConstants.moduleConfigs[0]),
                new ModuleIOTalonFX(SwerveConstants.moduleConfigs[1]),
                new ModuleIOTalonFX(SwerveConstants.moduleConfigs[2]),
                new ModuleIOTalonFX(SwerveConstants.moduleConfigs[3]));
        break;

      case SIM:
        /* Sim robot, instantiate physics sim IO implementations */

        /* create simulations */
        /* create simulation for pigeon2 IMU (different IMUs have different measurement erros) */
        this.gyroSimulation = GyroSimulation.createNavX2();
        /* create a swerve drive simulation */
        this.swerveDriveSimulation =
            new SwerveDriveSimulation(
                45,
                DriveConstants.TRACK_WIDTH,
                DriveConstants.WHEEL_BASE,
                DriveConstants.TRACK_WIDTH,
                DriveConstants.WHEEL_BASE,
                SwerveModuleSimulation.getModule(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    60,
                    DRIVE_WHEEL_TYPE.TIRE,
                    7.36),
                gyroSimulation,
                new Pose2d(3, 3, new Rotation2d()));
        SimulatedField.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
        driveSubsystem =
            new SwerveDrive(
                new GyroIOSim(
                    gyroSimulation), // GyroIOSim is a wrapper around gyro simulation, that reads
                // the simulation result
                /* ModuleIOSim are edited such that they also wraps around module simulations */
                new ModuleIOSim(swerveDriveSimulation.getModules()[0]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[1]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[2]),
                new ModuleIOSim(swerveDriveSimulation.getModules()[3]));

        SimulatedField.getInstance().resetFieldForAuto();
        resetFieldAndOdometryForAuto(new Pose2d(3, 3, new Rotation2d()));

        break;

      default:
        /* Replayed robot, disable IO implementations */

        /* physics simulations are also not needed */
        this.gyroSimulation = null;
        this.swerveDriveSimulation = null;
        // this.simulatedArena = null;
        driveSubsystem =
            new SwerveDrive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
  }

  private void resetFieldAndOdometryForAuto(Pose2d robotStartingPoseAtBlueAlliance) {
    final Pose2d startingPose = robotStartingPoseAtBlueAlliance;

    if (swerveDriveSimulation != null) {
      swerveDriveSimulation.setSimulationWorldPose(startingPose);
      SimulatedField.getInstance().resetFieldForAuto();
      updateFieldSimAndDisplay();
    }

    driveSubsystem.periodic();
    driveSubsystem.setPose(startingPose);
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxisCubed(DoubleSupplier supplierValue) {
    double value = supplierValue.getAsDouble();

    // Deadband
    value = deadband(value, HardwareConstants.DEADBAND_VALUE);

    // Cube the axis
    value = Math.copySign(value * value * value, value);

    return value;
  }

  private static double[] modifyAxisCubedPolar(DoubleSupplier xJoystick, DoubleSupplier yJoystick) {
    double xInput = deadband(xJoystick.getAsDouble(), HardwareConstants.DEADBAND_VALUE);
    double yInput = deadband(yJoystick.getAsDouble(), HardwareConstants.DEADBAND_VALUE);
    if (Math.abs(xInput) > 0 && Math.abs(yInput) > 0) {
      double theta = Math.atan(xInput / yInput);
      double hypotenuse = Math.sqrt(xInput * xInput + yInput * yInput);
      double cubedHypotenuse = Math.pow(hypotenuse, 3);
      xInput = Math.copySign(Math.sin(theta) * cubedHypotenuse, xInput);
      yInput = Math.copySign(Math.cos(theta) * cubedHypotenuse, yInput);
      return new double[] {xInput, yInput};
    }
    return new double[] {
      Math.copySign(xInput * xInput * xInput, xInput),
      Math.copySign(yInput * yInput * yInput, yInput)
    };
  }

  public void teleopInit() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    DoubleSupplier driverLeftStickX = driverController::getLeftX;
    DoubleSupplier driverLeftStickY = driverController::getLeftY;
    DoubleSupplier driverRightStickX = driverController::getRightX;
    DoubleSupplier driverLeftStick[] =
        new DoubleSupplier[] {
          () -> modifyAxisCubedPolar(driverLeftStickX, driverLeftStickY)[0],
          () -> modifyAxisCubedPolar(driverLeftStickX, driverLeftStickY)[1]
        };

    Trigger driverRightBumper = new Trigger(driverController::getRightBumper);
    Trigger driverLeftBumper = new Trigger(driverController::getLeftBumper);

    Command driveCommand =
        new DriveCommand(
            driveSubsystem,
            driverLeftStick[1],
            driverLeftStick[0],
            () -> modifyAxisCubed(driverRightStickX),
            () -> !driverRightBumper.getAsBoolean(),
            () -> driverLeftBumper.getAsBoolean());

    driveSubsystem.setDefaultCommand(driveCommand);
  }

  public Command getAutonomousCommand() {
    return new FeedForwardCharacterization(
        driveSubsystem,
        driveSubsystem::runCharacterization,
        driveSubsystem::getCharacterizationVelocity);
  }

  public void updateFieldSimAndDisplay() {
    if (swerveDriveSimulation == null) return;
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Notes",
        SimulatedField.getInstance().getGamePiecesByType("Note").toArray(Pose3d[]::new));
  }
}
