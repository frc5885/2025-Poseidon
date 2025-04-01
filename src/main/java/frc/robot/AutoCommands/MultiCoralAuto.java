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
import frc.robot.commands.WaitUntilCloseToCommand;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefLevel;
import frc.robot.util.FieldConstants.Side;
import java.util.List;
import java.util.function.Supplier;

public class MultiCoralAuto extends SequentialCommandGroup {
  private double kDistanceFromReefToWaitForHandoff = 0.5;
  private double kRadiansToReefToWaitForHandoff = Units.degreesToRadians(0);

  private double kLEDFlashDistance = 1.75;

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
                drive.setPose(FieldConstants.getSimInitialPose(side));
                feeder.simulateCoralFeed();
              }
            }));

    // start feeding immediately
    addCommands(feeder.startFeederCmd());

    // for each branch, drive to reef, score, then intake
    branches.stream()
        .forEach(
            branchNum -> {
              Pose3d branchPose3d = FieldConstants.getBranchPose3d(branchNum, ReefLevel.L4);
              addCommands(
                  driveToReefAndHandoffCommand(
                      drive,
                      superStructure,
                      feeder,
                      endEffector,
                      () -> branchPose3d.toPose2d(),
                      () -> branchNum),
                  new AutoScoreCoralAtBranchCommand(
                      drive,
                      superStructure,
                      endEffector,
                      () -> branchNum,
                      () -> ReefLevel.L4.ordinal(),
                      () -> false),
                  startFeederAndDriveToLoadingStationCommand(
                      drive,
                      superStructure,
                      feeder,
                      () -> FieldConstants.getIntakePose(side),
                      () -> branchNum));
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
      Supplier<Pose2d> targetPoseSupplier,
      Supplier<Integer> branchNumSupplier) {
    return new ParallelDeadlineGroup(
        // wait for handoff ready, then handoff
        new WaitUntilCommand(() -> feeder.getIsHandoffReady())
            .andThen(new CoralHandoffCommand(superStructure, feeder, endEffector))
            .withTimeout(3.0),
        // drive to reef
        DriveCommands.pidToPose(
            drive,
            () ->
                targetPoseSupplier
                    .get()
                    .transformBy(
                        new Transform2d(
                            -kDistanceFromReefToWaitForHandoff,
                            0,
                            new Rotation2d(kRadiansToReefToWaitForHandoff))),
            branchNumSupplier));
  }

  private Command startFeederAndDriveToLoadingStationCommand(
      Drive drive,
      SuperStructure superStructure,
      Feeder feeder,
      Supplier<Pose2d> intakePoseSupplier,
      Supplier<Integer> branchNumSupplier) {
    return feeder
        .startFeederCmd()
        .andThen(
            new ParallelCommandGroup(
                new ResetSuperStructureCommand(drive, superStructure, false),
                DriveCommands.auto_basicPathplannerToPose(drive, intakePoseSupplier),
                new WaitUntilCloseToCommand(
                        () -> drive.getPose(), intakePoseSupplier, kLEDFlashDistance)
                    .andThen(new InstantCommand(() -> LEDSubsystem.getInstance().flashGreen()))));
  }
}
