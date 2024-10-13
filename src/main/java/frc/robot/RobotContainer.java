package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.extras.simulation.SimulatedField;
import frc.robot.extras.simulation.physicsSim.GyroSimulation;
import frc.robot.extras.simulation.physicsSim.SwerveDriveSimulation;
import frc.robot.extras.simulation.physicsSim.SwerveModuleSimulation;
import frc.robot.extras.simulation.physicsSim.SwerveModuleSimulation.DRIVE_WHEEL_TYPE;
import frc.robot.subsystems.swerve.SwerveConstants;
// import frc.robot.extras.characterization.WheelRadiusCharacterization;
// import frc.robot.extras.characterization.WheelRadiusCharacterization.Direction;
// import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.gyroIO.GyroIO;
import frc.robot.subsystems.swerve.gyroIO.GyroIONavX;
import frc.robot.subsystems.swerve.gyroIO.GyroIOSim;
import frc.robot.subsystems.swerve.moduleIO.ModuleIO;
import frc.robot.subsystems.swerve.moduleIO.ModuleIOSim;
import frc.robot.subsystems.swerve.moduleIO.ModuleIOTalonFX;

import java.util.List;
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
    SmarterDashboardRegistry.initialize();
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
                    DCMotor.getKrakenX60(1), DCMotor.getFalcon500(1), 60, DRIVE_WHEEL_TYPE.TIRE, 7.36),
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
                        resetFieldAndOdometryForAuto(new Pose2d(3,3, new Rotation2d()));

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
    SmarterDashboardRegistry.initialize();
  }

  // public void intakeCallback(boolean hasNote) {
  //   if (hasNote) {
  //     driverController.setRumble(RumbleType.kBothRumble, 0.1);
  //     operatorController.setRumble(RumbleType.kBothRumble, 1);
  //   } else {
  //     driverController.setRumble(RumbleType.kBothRumble, 0);
  //     operatorController.setRumble(RumbleType.kBothRumble, 0);
  //   }
  // }

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
    Trigger driverRightDirectionPad = new Trigger(() -> driverController.getPOV() == 90);
    Trigger driverDownDirectionPad = new Trigger(() -> driverController.getPOV() == 180);
    Trigger driverLeftDirectionPad = new Trigger(() -> driverController.getPOV() == 270);

    // // autodrive
    // Trigger driverAButton = new Trigger(driverController::getAButton);

    // // intake
    // Trigger operatorLeftTrigger = new Trigger(()->operatorController.getLeftTriggerAxis() > 0.2);
    // Trigger operatorLeftBumper = new Trigger(operatorController::getLeftBumper);
    // // amp and speaker
    // Trigger operatorBButton = new Trigger(operatorController::getBButton);
    // Trigger operatorRightBumper = new Trigger(operatorController::getRightBumper);
    // Trigger operatorRightTrigger = new Trigger(()->operatorController.getRightTriggerAxis() >
    // 0.2);
    // Trigger driverRightTrigger = new Trigger(()->driverController.getRightTriggerAxis() > 0.2);

    // // manual pivot and intake rollers
    // Trigger operatorAButton = new Trigger(operatorController::getAButton);
    // Trigger operatorXButton = new Trigger(operatorController::getXButton);
    // Trigger operatorYButton = new Trigger(operatorController::getYButton);
    // DoubleSupplier operatorRightStickY = operatorController::getRightY;
    // // unused
    // Trigger operatorUpDirectionPad = new Trigger(()->operatorController.getPOV() == 0);
    // Trigger operatorLeftDirectionPad = new Trigger(()->operatorController.getPOV() == 270);
    // Trigger operatorDownDirectionPad = new Trigger(()->operatorController.getPOV() == 180);
    // Trigger driverLeftTrigger = new Trigger(()->driverController.getLeftTriggerAxis() > 0.2);
    Trigger driverLeftBumper = new Trigger(driverController::getLeftBumper);
    // Trigger driverBButton = new Trigger(driverController::getBButton);
    // Trigger driverYButton = new Trigger(driverController::getYButton);
    // DoubleSupplier operatorLeftStickY = operatorController::getLeftY;

    // //DRIVER BUTTONS

    // // driving

    Command driveCommand =
        new DriveCommand(
            driveSubsystem,
            driverLeftStick[1],
            driverLeftStick[0],
            () -> modifyAxisCubed(driverRightStickX),
            () -> !driverRightBumper.getAsBoolean(),
            () -> driverLeftBumper.getAsBoolean());

    driveSubsystem.setDefaultCommand(driveCommand);
    // // shooterSubsystem.setDefaultCommand(new FlywheelSpinUpAuto(shooterSubsystem,
    // visionSubsystem));

    // driverLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem, false, ledSubsystem, this::intakeCallback));
    // driverLeftTrigger.whileFalse(new TowerIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem, false, ledSubsystem, this::intakeCallback).withTimeout(0.3));
    // // Amp Lineup
    // driverAButton.whileTrue(new AutoAlignWithAmp(driveSubsystem, visionSubsystem));
    // // Spinup for shoot
    // driverRightTrigger.whileTrue(new SpinUpForSpeaker(driveSubsystem, shooterSubsystem,
    // pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper,
    // ledSubsystem));

    // // driverLeftBumper.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem,
    // pivotSubsystem, visionSubsystem, driverLeftStickX, operatorLeftStickY, driverRightBumper,
    // ledSubsystem));
    // // driverRightTrigger.whileTrue(new ShootWhileMove(driveSubsystem, shooterSubsystem,
    // pivotSubsystem, visionSubsystem, driverLeftStick, driverYButton, ledSubsystem));

    // // Resets the robot angle in the odometry, factors in which alliance the robot is on
    // driverRightDirectionPad.onTrue(new InstantCommand(() -> driveSubsystem.resetOdometry(new
    // Pose2d(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY(),
    //       Rotation2d.fromDegrees(driveSubsystem.getAllianceAngleOffset())))));
    // // // Reset robot odometry based on vision pose measurement from april tags
    // driverLeftDirectionPad.onTrue(new InstantCommand(() ->
    // driveSubsystem.resetOdometry(visionSubsystem.getLastSeenPose())));
    // // driverLeftDpad.onTrue(new InstantCommand(() -> driveSubsystem.resetOdometry(new
    // Pose2d(15.251774787902832, 5.573054313659668, Rotation2d.fromRadians(3.14159265)))));
    // // driverBButton.whileTrue(new ShootPass(driveSubsystem, shooterSubsystem, pivotSubsystem,
    // visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper, ledSubsystem));

    // // driverXButton.
    // driverBButton.whileTrue(new ShootPass(driveSubsystem, shooterSubsystem, pivotSubsystem,
    // visionSubsystem, driverLeftStickY, operatorLeftStickY, driverYButton, ledSubsystem));
    // // driverDownDirectionPad.whileTrue(new IntakeFromShooter(shooterSubsystem,
    // intakeSubsystem));
    // // driverYButton.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem,
    // visionSubsystem, driverLeftStickX, operatorLeftStickY, driverRightBumper, ledSubsystem));
    // // OPERATOR BUTTONS

    // // speaker
    // operatorRightTrigger.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem,
    // pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper,
    // ledSubsystem));
    // // amp
    // operatorRightBumper.whileTrue(new ShootAmp(shooterSubsystem, pivotSubsystem, ledSubsystem,
    // operatorBButton));
    // // fender shot
    // operatorUpDirectionPad.whileTrue(new SubwooferShot(driveSubsystem, shooterSubsystem,
    // pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightStickX,
    // driverRightBumper, ledSubsystem));
    // // intake (aka SUCC_BUTTON)
    // operatorLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem, false, ledSubsystem, this::intakeCallback));
    // operatorLeftTrigger.whileFalse(new TowerIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem, false, ledSubsystem, this::intakeCallback).withTimeout(0.2));
    // // outtake (aka UNSUCC_BUTTON)
    // operatorLeftBumper.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem, true, ledSubsystem, this::intakeCallback));
    // // manual pivot (possible climb, unlikely)
    // operatorAButton.whileTrue(new ManualPivot(pivotSubsystem,
    // ()->modifyAxisCubed(operatorRightStickY)));
    // operatorDownDirectionPad.whileTrue(new ManualPivot(pivotSubsystem, ()->-0.2));
    // // manual rollers
    // operatorYButton.whileTrue(new ManualIntake(intakeSubsystem, true));
    // operatorXButton.whileTrue(new ManualIntake(intakeSubsystem, false));

    // operatorBButton.onTrue(new StopShooterAndIntake(intakeSubsystem, pivotSubsystem,
    // shooterSubsystem));
  }

  public Command getAutonomousCommand() {
    SmarterDashboardRegistry.initialize();
    // Resets the pose factoring in the robot side
    // This is just a failsafe, pose should be reset at the beginning of auto
    // driveSubsystem.resetOdometry(new Pose2d(driveSubsystem.getPose().getX(),
    // driveSubsystem.getPose().getY(),
    //   Rotation2d.fromDegrees(driveSubsystem.getAllianceAngleOffset())));
    // return autoChooser.getSelected();
    return null;
  }

  public void updateFieldSimAndDisplay() {
    if (swerveDriveSimulation == null)
        return;
    Logger.recordOutput("FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput("FieldSimulation/Notes", SimulatedField.getInstance().getGamePiecesByType("Note").toArray(Pose3d[]::new));
}
}
