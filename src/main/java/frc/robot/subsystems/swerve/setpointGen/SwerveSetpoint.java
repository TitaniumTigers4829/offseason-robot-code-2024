// // Copyright (c) 2024 FRC 6328
// // http://github.com/Mechanical-Advantage
// //
// // Use of this source code is governed by an MIT-style
// // license that can be found in the LICENSE file at
// // the root directory of this project.

// package frc.robot.subsystems.swerve.setpointGen;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveModuleState;

// public record SwerveSetpoint(ChassisSpeeds chassisSpeeds, AdvancedSwerveModuleState[] moduleStates) {}

package frc.robot.subsystems.swerve.setpointGen;

import frc.robot.subsystems.swerve.setpointGen.AdvancedSwerveModuleState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public record SwerveSetpoint(
    ChassisSpeeds chassisSpeeds,
    // @FixedSizeArray( size = 4 )
    AdvancedSwerveModuleState[] moduleStates
) implements StructSerializable {
    public static SwerveSetpoint zeroed() {
        return new SwerveSetpoint(new ChassisSpeeds(), new AdvancedSwerveModuleState[] {
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0)
        });
    }

    public static final Struct<SwerveSetpoint> struct = SwerveSetpoint.struct;
}