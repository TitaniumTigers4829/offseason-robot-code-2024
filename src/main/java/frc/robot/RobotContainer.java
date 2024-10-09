package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.LEDConstants.LEDProcess;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.shooter.SpinupFlywheel;
import frc.robot.extras.SmarterDashboardRegistry;
// import frc.robot.extras.characterization.WheelRadiusCharacterization;
// import frc.robot.extras.characterization.WheelRadiusCharacterization.Direction;
// import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.GyroIONavX;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;

public class RobotContainer {

  // private final Vision visionSubsystem;
  private final Drive driveSubsystem;
  private final Flywheel flywheelSubsystem;
  private final XboxController driverController = new XboxController(0);
  
  public RobotContainer() {
    SmarterDashboardRegistry.initialize();
    // visionSubsystem = new Vision();
    driveSubsystem = new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
    flywheelSubsystem = new Flywheel(new FlywheelIOTalonFX());
    
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
      return new double[]{xInput, yInput};
    }
    return new double[]{ Math.copySign(xInput * xInput * xInput, xInput),  Math.copySign(yInput * yInput * yInput, yInput)};
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
    DoubleSupplier driverLeftStick[] = new DoubleSupplier[]{()->modifyAxisCubedPolar(driverLeftStickX, driverLeftStickY)[0], ()->modifyAxisCubedPolar(driverLeftStickX, driverLeftStickY)[1]};

    Trigger driverRightBumper = new Trigger(driverController::getRightBumper);
    Trigger driverRightDirectionPad = new Trigger(()->driverController.getPOV() == 90);
    Trigger driverDownDirectionPad = new Trigger(()->driverController.getPOV() == 180);
    Trigger driverLeftDirectionPad = new Trigger(()->driverController.getPOV() == 270);
    // // autodrive
    // Trigger driverAButton = new Trigger(driverController::getAButton);

    // // intake
    // Trigger operatorLeftTrigger = new Trigger(()->operatorController.getLeftTriggerAxis() > 0.2);
    // Trigger operatorLeftBumper = new Trigger(operatorController::getLeftBumper);
    // // amp and speaker
    // Trigger operatorBButton = new Trigger(operatorController::getBButton);
    // Trigger operatorRightBumper = new Trigger(operatorController::getRightBumper);
    // Trigger operatorRightTrigger = new Trigger(()->operatorController.getRightTriggerAxis() > 0.2);
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

    Command driveCommand = new DriveCommand(driveSubsystem,
      driverLeftStick[1],
      driverLeftStick[0],
      () -> modifyAxisCubed(driverRightStickX),
      () -> !driverRightBumper.getAsBoolean(),
      () -> driverLeftBumper.getAsBoolean()
    );

    driveSubsystem.setDefaultCommand(driveCommand);
    // // shooterSubsystem.setDefaultCommand(new FlywheelSpinUpAuto(shooterSubsystem, visionSubsystem));

    // driverLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, ledSubsystem, this::intakeCallback));
    // driverLeftTrigger.whileFalse(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, ledSubsystem, this::intakeCallback).withTimeout(0.3));
    // // Amp Lineup
    // driverAButton.whileTrue(new AutoAlignWithAmp(driveSubsystem, visionSubsystem));
    // // Spinup for shoot
    // driverRightTrigger.whileTrue(new SpinUpForSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper, ledSubsystem));
    
    // // driverLeftBumper.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, operatorLeftStickY, driverRightBumper, ledSubsystem));
    // // driverRightTrigger.whileTrue(new ShootWhileMove(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStick, driverYButton, ledSubsystem));

    // // Resets the robot angle in the odometry, factors in which alliance the robot is on
    // driverRightDirectionPad.onTrue(new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY(), 
    //       Rotation2d.fromDegrees(driveSubsystem.getAllianceAngleOffset())))));
    // // // Reset robot odometry based on vision pose measurement from april tags
    // driverLeftDirectionPad.onTrue(new InstantCommand(() -> driveSubsystem.resetOdometry(visionSubsystem.getLastSeenPose())));
    // // driverLeftDpad.onTrue(new InstantCommand(() -> driveSubsystem.resetOdometry(new Pose2d(15.251774787902832, 5.573054313659668, Rotation2d.fromRadians(3.14159265)))));
    // // driverBButton.whileTrue(new ShootPass(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper, ledSubsystem));

    // // driverXButton.
    // driverBButton.whileTrue(new ShootPass(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickY, operatorLeftStickY, driverYButton, ledSubsystem));
    // // driverDownDirectionPad.whileTrue(new IntakeFromShooter(shooterSubsystem, intakeSubsystem));
    // // driverYButton.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, operatorLeftStickY, driverRightBumper, ledSubsystem));
    // // OPERATOR BUTTONS

    // // speaker
    // operatorRightTrigger.whileTrue(new ShootSpeaker(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightBumper, ledSubsystem));
    // // amp
    // operatorRightBumper.whileTrue(new ShootAmp(shooterSubsystem, pivotSubsystem, ledSubsystem, operatorBButton));
    // // fender shot
    // operatorUpDirectionPad.whileTrue(new SubwooferShot(driveSubsystem, shooterSubsystem, pivotSubsystem, visionSubsystem, driverLeftStickX, driverLeftStickY, driverRightStickX, driverRightBumper, ledSubsystem));
    // // intake (aka SUCC_BUTTON)
    // operatorLeftTrigger.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, ledSubsystem, this::intakeCallback));
    // operatorLeftTrigger.whileFalse(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, false, ledSubsystem, this::intakeCallback).withTimeout(0.2));
    // // outtake (aka UNSUCC_BUTTON)
    // operatorLeftBumper.whileTrue(new TowerIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem, true, ledSubsystem, this::intakeCallback));
    // // manual pivot (possible climb, unlikely)
    // operatorAButton.whileTrue(new ManualPivot(pivotSubsystem, ()->modifyAxisCubed(operatorRightStickY)));
    // operatorDownDirectionPad.whileTrue(new ManualPivot(pivotSubsystem, ()->-0.2));
    // // manual rollers
    // operatorYButton.whileTrue(new ManualIntake(intakeSubsystem, true));
    // operatorXButton.whileTrue(new ManualIntake(intakeSubsystem, false));
    
    // operatorBButton.onTrue(new StopShooterAndIntake(intakeSubsystem, pivotSubsystem, shooterSubsystem));
  }

  public Command getAutonomousCommand() {
    SmarterDashboardRegistry.initialize();
    // Resets the pose factoring in the robot side
    // This is just a failsafe, pose should be reset at the beginning of auto
    // driveSubsystem.resetOdometry(new Pose2d(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY(), 
    //   Rotation2d.fromDegrees(driveSubsystem.getAllianceAngleOffset())));
    // return autoChooser.getSelected();
    return null;
  }
}