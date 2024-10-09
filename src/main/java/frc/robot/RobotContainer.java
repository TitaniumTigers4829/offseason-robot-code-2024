package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HardwareConstants;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.extras.SmarterDashboardRegistry;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.gyroIO.GyroIONavX;
import frc.robot.subsystems.swerve.moduleIO.ModuleIOTalonFX;
import java.util.function.DoubleSupplier;

public class RobotContainer {

  private final SwerveDrive driveSubsystem;
  private final XboxController driverController = new XboxController(0);

  public RobotContainer() {
    SmarterDashboardRegistry.initialize();
    driveSubsystem =
        new SwerveDrive(
            new GyroIONavX(),
            new ModuleIOTalonFX(SwerveConstants.moduleConfigs[0]),
            new ModuleIOTalonFX(SwerveConstants.moduleConfigs[1]),
            new ModuleIOTalonFX(SwerveConstants.moduleConfigs[2]),
            new ModuleIOTalonFX(SwerveConstants.moduleConfigs[3]));
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
    SmarterDashboardRegistry.initialize();
    return null;
  }
}
