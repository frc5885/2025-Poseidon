// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.FieldConstants;
import java.util.List;

public class AutoIntakeCoralStationCommand extends SequentialCommandGroup {
  private List<Pose2d> stationPoses =
      List.of(
          FieldConstants.CoralStation.leftCenterFace, FieldConstants.CoralStation.rightCenterFace);
  private Pose2d closeTransitionPose;
  private Pose2d farTransitionPose;
  private final double kTransitionDistance = 0.9;

  public AutoIntakeCoralStationCommand(
      Drive drive, SuperStructure superStructure, EndEffector endEffector) {
    addCommands(
        new InstantCommand(
            () -> {
              closeTransitionPose =
                  drive
                      .getPose()
                      .nearest(stationPoses)
                      .transformBy(new Transform2d(kTransitionDistance, 0.0, new Rotation2d()));
              farTransitionPose =
                  closeTransitionPose.transformBy(
                      new Transform2d(kTransitionDistance, 0.0, new Rotation2d()));
            }),
        new ParallelCommandGroup(
            new DriveToPoseCommand(
                drive,
                () ->
                    farTransitionPose.transformBy(
                        new Transform2d(0.0, 0.0, new Rotation2d(Math.PI))),
                DriveConstants.kDistanceTolerance,
                DriveConstants.kRotationTolerance,
                false),
            new SuperStructureCommand(superStructure, () -> SuperStructureState.CORAL_STATION)),
        new ParallelCommandGroup(
            DriveCommands.preciseChassisAlign(
                drive,
                () ->
                    closeTransitionPose.transformBy(
                        new Transform2d(0.0, 0.0, new Rotation2d(Math.PI)))),
            new IntakeCoralStationCommand(endEffector)));
  }
}
