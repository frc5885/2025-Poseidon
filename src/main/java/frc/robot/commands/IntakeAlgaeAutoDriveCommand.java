// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.FieldConstants;
import java.util.function.Supplier;

public class IntakeAlgaeAutoDriveCommand extends SequentialCommandGroup {
  private Pose2d transitionPose = new Pose2d();
  private SuperStructureState state = null;

  public IntakeAlgaeAutoDriveCommand(
      Drive drive,
      SuperStructure superStructure,
      EndEffector endEffector,
      Supplier<Pose2d> targetPose) {

    addCommands(
        new InstantCommand(
            () -> {
              transitionPose =
                  targetPose.get().transformBy(new Transform2d(-0.35, 0.0, new Rotation2d()));
              for (int i = 0; i < FieldConstants.Reef.centerFaces.length; i++) {
                if (FieldConstants.Reef.centerFaces[i].equals(targetPose.get())) {
                  state = FieldConstants.Reef.AlgaeLevel[i];
                  break;
                }
              }
            }),
        new ParallelCommandGroup(
            new RunCommand(
                    () -> superStructure.setSuperStructureGoal(state).schedule(), superStructure)
                .until(superStructure::isFinalGoalAchieved),
            new RunCommand(
                    () -> drive.getDriveToPoseCommand(() -> transitionPose).schedule(), drive)
                .until(
                    () ->
                        drive
                                    .getPose()
                                    .getTranslation()
                                    .getDistance(transitionPose.getTranslation())
                                < DriveConstants.kDistanceTolerance
                            && drive
                                    .getPose()
                                    .getRotation()
                                    .minus(transitionPose.getRotation())
                                    .getDegrees()
                                < DriveConstants.kRotationTolerance)),
        new RunCommand(() -> DriveCommands.preciseChassisAlign(drive, targetPose).schedule())
            .until(
                () ->
                    drive.getPose().getTranslation().getDistance(targetPose.get().getTranslation())
                            < DriveConstants.kDistanceTolerance
                        && drive
                                .getPose()
                                .getRotation()
                                .minus(targetPose.get().getRotation())
                                .getDegrees()
                            < DriveConstants.kRotationTolerance)
            .finallyDo(
                () -> System.out.println("================================================")),
        new IntakeAlgaeCommand(endEffector));
  }
}
