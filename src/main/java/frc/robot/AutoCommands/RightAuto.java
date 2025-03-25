// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.CoralHandoffCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ResetSuperStructureCommand;
import frc.robot.commands.SuperStructureCommand;
import frc.robot.commands.WaitUntilCloseToCommand;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefLevel;
import frc.robot.util.GamePieces.GamePieceVisualizer;
import java.util.ArrayList;
import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightAuto extends SequentialCommandGroup {
  private Pose2d intakePose = new Pose2d(1.22, 0.88, Rotation2d.fromDegrees(54.27));
  private Pose2d initialPose = new Pose2d(7.3, 1.6, Rotation2d.fromRadians(Math.PI));

  // Branches to score coral at
  private ArrayList<Integer> branches = new ArrayList<>(List.of(9, 10, 11));

  public RightAuto(
      Drive drive, SuperStructure superStructure, Feeder feeder, EndEffector endEffector) {

    // Setup command for simulation and alliance flipping
    addCommands(
        new InstantCommand(
            () -> {
              intakePose = AllianceFlipUtil.apply(intakePose);
              if (Constants.kCurrentMode.equals(Mode.SIM)) {
                drive.setPose(AllianceFlipUtil.apply(initialPose));
              }
            }));

    // first branch
    int firstBranchNum = branches.get(0);
    // drive to pose and move superstructure to scoring, and then score the coral
    addCommands(
        new ParallelCommandGroup(
                DriveCommands.auto_optimalTrajectoryReefAlign(
                    drive,
                    () ->
                        FieldConstants.Reef.branchPositions
                            .get(firstBranchNum)
                            .get(ReefLevel.L4)
                            .toPose2d()),
                new SuperStructureCommand(
                    superStructure, () -> getScoringSuperStructureState(ReefLevel.L4)))
            .andThen(
                new SuperStructureCommand(
                    superStructure, () -> getScoredSuperStructureState(ReefLevel.L4)))
            .andThen(() -> GamePieceVisualizer.setHasCoral(false)));

    branches.stream()
        // skip the first branch because we already created a command for it above
        .skip(1)
        .forEach(
            branchNum -> {

              // actual command ===========================================
              // reset superstructure, wait for handoff ready, do handoff, and move to
              // superstructure scoring position
              Command ssCmd =
                  new ResetSuperStructureCommand(drive, superStructure, false)
                      .andThen(feeder.startFeederCmd())
                      .andThen(
                          new WaitUntilCommand(() -> feeder.getIsHandoffReady())
                              .andThen(new CoralHandoffCommand(superStructure, feeder, endEffector))
                              .andThen(
                                  new SuperStructureCommand(
                                      superStructure,
                                      () -> getScoringSuperStructureState(ReefLevel.L4))));

              // along with drive to intake and then drive to reef
              Command chassisCmd =
                  DriveCommands.auto_basicPathplannerToPose(drive, () -> intakePose, false)
                      .alongWith(
                          new WaitUntilCloseToCommand(() -> drive.getPose(), () -> intakePose, 1.75)
                              .andThen(
                                  new InstantCommand(
                                      () -> LEDSubsystem.getInstance().flashGreen())))
                      .andThen(
                          DriveCommands.auto_optimalTrajectoryReefAlign(
                              drive,
                              () ->
                                  FieldConstants.Reef.branchPositions
                                      .get(branchNum)
                                      .get(ReefLevel.L4)
                                      .toPose2d()));

              // Score coral
              Command scoreCmd =
                  new SuperStructureCommand(
                          superStructure, () -> getScoredSuperStructureState(ReefLevel.L4))
                      .alongWith(
                          new InstantCommand(
                              () -> endEffector.runEndEffectorOuttake(), endEffector))
                      .andThen(
                          () -> {
                            endEffector.stopEndEffector();
                            GamePieceVisualizer.setHasCoral(false);
                          });

              // combination of ss and chassis in parallel, then score
              addCommands(ssCmd.alongWith(chassisCmd), scoreCmd);
              // end of actual command ===========================================
            });
  }

  private SuperStructureState getScoringSuperStructureState(ReefLevel reefLevel) {
    switch (reefLevel) {
      case L1:
        return SuperStructureState.SCORE_CORAL_L1;
      case L2:
        return SuperStructureState.SCORE_CORAL_L2;
      case L3:
        return SuperStructureState.SCORE_CORAL_L3;
      case L4:
        return SuperStructureState.SCORE_CORAL_L4;
      default:
        return SuperStructureState.SCORE_CORAL_L4;
    }
  }

  private SuperStructureState getScoredSuperStructureState(ReefLevel reefLevel) {
    switch (reefLevel) {
      case L1:
        return SuperStructureState.IDLE;
      case L2:
        return SuperStructureState.SCORED_CORAL_L2;
      case L3:
        return SuperStructureState.SCORED_CORAL_L3;
      case L4:
        return SuperStructureState.SCORED_CORAL_L4;
      default:
        return SuperStructureState.SCORED_CORAL_L4;
    }
  }
}
