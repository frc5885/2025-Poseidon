// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.CoralHandoffCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ResetSuperStructureCommand;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefLevel;
import frc.robot.util.FieldConstants.Side;
import java.util.List;
import java.util.function.Supplier;

public class MultiCoralAuto extends SequentialCommandGroup {
  // intake pose for blue right side, gets flipped if needed later
  private Pose2d intakePose = new Pose2d(1.22, 0.88, Rotation2d.fromDegrees(54.27));
  // initial pose for blue right side, gets flipped if needed later (only used in SIM)
  private Pose2d initialPose = new Pose2d(7.3, 1.6, Rotation2d.fromRadians(Math.PI));

  private double kDistanceFromReefToWaitForHandoff = 0.5;
  private double kRadiansToReefToWaitForHandoff = Units.degreesToRadians(0);

  public MultiCoralAuto(
      Drive drive,
      SuperStructure superStructure,
      Feeder feeder,
      EndEffector endEffector,
      Side side,
      List<Integer> branches) {

    // Sim setup
    addCommands(
        new InstantCommand(
            () -> {
              if (Constants.kCurrentMode.equals(Mode.SIM)) {
                Pose2d flippedInitialPose = flipAndMirrorPose(initialPose, side);
                drive.setPose(flippedInitialPose);
                feeder.simulateCoralFeed();
              }
            }));

    // start feeding immediately
    addCommands(feeder.startFeederCmd());

    // for each branch, drive to reef, score, then intake
    branches.stream()
        .forEach(
            branchNum -> {
              Pose2d flippedIntakePose = flipAndMirrorPose(intakePose, side);
              Pose3d branchPose3d =
                  FieldConstants.Reef.branchPositions.get(branchNum).get(ReefLevel.L4);
              addCommands(
                  driveToReefAndHandoffCommand(
                      drive, superStructure, feeder, endEffector, () -> branchPose3d.toPose2d()),
                  new AutoScoreCoralAtBranchCommand(
                      drive, superStructure, endEffector, () -> branchPose3d),
                  startFeederAndDriveToLoadingStationCommand(
                      drive, superStructure, feeder, () -> flippedIntakePose));
            });
  }

  /**
   * Drive to reef and handoff whenever ready. Returns as soon as the handoff is done, regardless of
   * the robot's pose at that time
   */
  private Command driveToReefAndHandoffCommand(
      Drive drive,
      SuperStructure superStructure,
      Feeder feeder,
      EndEffector endEffector,
      Supplier<Pose2d> targetPoseSupplier) {
    return new ParallelDeadlineGroup(
        // wait for handoff ready, then handoff
        new WaitUntilCommand(() -> feeder.getIsHandoffReady())
            .andThen(new CoralHandoffCommand(superStructure, feeder, endEffector)),
        // drive to reef
        DriveCommands.auto_optimalTrajectoryReefAlign(
            drive,
            () ->
                targetPoseSupplier
                    .get()
                    .transformBy(
                        new Transform2d(
                            -kDistanceFromReefToWaitForHandoff,
                            0,
                            new Rotation2d(kRadiansToReefToWaitForHandoff)))));
  }

  private Command startFeederAndDriveToLoadingStationCommand(
      Drive drive,
      SuperStructure superStructure,
      Feeder feeder,
      Supplier<Pose2d> intakePoseSupplier) {
    return feeder
        .startFeederCmd()
        .andThen(
            new ParallelCommandGroup(
                new ResetSuperStructureCommand(drive, superStructure, false),
                DriveCommands.auto_basicPathplannerToPose(drive, intakePoseSupplier, false)));
  }

  private Pose2d flipAndMirrorPose(Pose2d poseBlueRight, Side side) {
    Pose2d returnPose = new Pose2d();
    returnPose = AllianceFlipUtil.apply(poseBlueRight);

    // mirror pose for left vs right
    if (side == Side.LEFT) {
      returnPose =
          new Pose2d(
              returnPose.getX(),
              FieldConstants.fieldWidth - returnPose.getY(),
              Rotation2d.fromRadians(-returnPose.getRotation().getRadians()));
    }
    return returnPose;
  }
}
