// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.FieldConstants;
import java.util.List;
import java.util.function.Supplier;

public class IntakeAlgaeAutoDriveCommand extends SequentialCommandGroup {
  public IntakeAlgaeAutoDriveCommand(
      Drive drive,
      SuperStructure superStructure,
      EndEffector endEffector,
      Supplier<Pose2d> robotPose) {
    Pose2d targetPose = robotPose.get().nearest(List.of(FieldConstants.Reef.centerFaces));
    Pose2d transitionPose2d = targetPose.transformBy(new Transform2d(-0.6, 0.0, new Rotation2d()));
    SuperStructureState state = SuperStructureState.STOWED;
    for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
      if (FieldConstants.Reef.centerFaces[i].equals(targetPose)) {
        state = FieldConstants.Reef.AlgaeLevel[i];
        break;
      }
    }

    addCommands(
        new ParallelCommandGroup(
            new SuperStructureCommand(superStructure, state),
            new DriveToPoseCommand(
                drive,
                () -> transitionPose2d,
                DriveConstants.kDistanceTolerance,
                DriveConstants.kRotationTolerance)),
        DriveCommands.preciseChassisAlign(drive, () -> targetPose),
        new IntakeAlgaeCommand(endEffector));
  }
}
