// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class ScoreAlgaeNet extends SequentialCommandGroup {
  public ScoreAlgaeNet(Drive drive, SuperStructure superStructure, EndEffector endEffector) {
    addCommands(
        // TODO requires elevator stability
        new DriveToPoseCommand(
            drive,
            () -> new Pose2d(7.0, drive.getPose().getY(), new Rotation2d()),
            DriveConstants.kDistanceTolerance,
            DriveConstants.kRotationTolerance),
        new SuperStructureCommand(superStructure, () -> SuperStructureState.SCORE_ALGAE_NET),
        DriveCommands.preciseChassisAlign(
            drive, () -> new Pose2d(7.8, drive.getPose().getY(), new Rotation2d())));
  }
}
