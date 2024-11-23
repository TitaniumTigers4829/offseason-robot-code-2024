// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.setpointGen;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record SwerveSetpoint(
    ChassisSpeeds chassisSpeeds,
    AdvancedSwerveModuleState[] moduleStates
) {
    public static SwerveSetpoint zeroed() {
        return new SwerveSetpoint(new ChassisSpeeds(), new AdvancedSwerveModuleState[] {
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0),
            new AdvancedSwerveModuleState(0, Rotation2d.kZero, 0, 0)
        });
    }
}
