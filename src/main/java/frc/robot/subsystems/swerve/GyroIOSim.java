// package frc.robot.subsystems.swerve;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Twist2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.util.Units;
// import frc.robot.Constants.DriveConstants;

// public class GyroIOSim implements GyroIO {
//     Drive drive;

//     private Pose2d simOdometry = new Pose2d();
//     double[] lastModulePositionsRad = {0, 0, 0, 0};

//     public GyroIOSim(Drive drivetrainSubsystem) {
//         this.drive = drivetrainSubsystem;
//     }

//     @Override
//     public void updateInputs(GyroIOInputs inputs) {
//         calcAngle();
//         inputs.yaw = simOdometry.getRotation().getRadians();
//     }

//     private void calcAngle() {
//         // drive.swerveModule;

//         Rotation2d[] turnPositions = new Rotation2d[4];
//         for (int i = 0; i < 4; i++) {
//             turnPositions[i] = drive.swerveModulePositions;
//         }

//         SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
//         for (int i = 0; i < 4; i++) {
//             measuredStatesDiff[i] = new SwerveModuleState(
//                     (drive.getSwerveModules()[i].getPosition().distanceMeters -
// lastModulePositionsRad[i])
//                             * Units.inchesToMeters(2),
//                     turnPositions[i]);
//             lastModulePositionsRad[i] = drive.getModulePositions().distanceMeters;
//         }

//         simOdometry = simOdometry.exp(new Twist2d(
//
// DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(measuredStatesDiff).vxMetersPerSecond,
//
// DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(measuredStatesDiff).vyMetersPerSecond,
//
// DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(measuredStatesDiff).omegaRadiansPerSecond));
//     }
// }
