// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.util.FieldConstants.ReefLevel;
import frc.robot.util.GamePieces.GamePieceVisualizer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCoralCommand extends SequentialCommandGroup {

  private SuperStructureState scoredState;

  public PlaceCoralCommand(
      ReefLevel reefLevel, SuperStructure superStructure, EndEffector endEffector) {
    addCommands(
        // set up ending state
        new InstantCommand(
            () ->
                scoredState =
                    switch (reefLevel) {
                      case L1 -> SuperStructureState.SCORE_CORAL_L1;
                      case L2 -> SuperStructureState.SCORED_CORAL_L2;
                      case L3 -> SuperStructureState.SCORED_CORAL_L3;
                      case L4 -> SuperStructureState.SCORED_CORAL_L4;
                      default -> SuperStructureState.IDLE;
                    }),
        new ConditionalCommand(
                // L1
                new InstantCommand(() -> endEffector.runEndEffectorOuttake())
                    .andThen(new WaitCommand(0.5)),
                // L2, L3, L4
                new ParallelCommandGroup(
                    // move to scored state, but wait 0.1s before running outtake
                    new SuperStructureCommand(superStructure, () -> scoredState),
                    new SequentialCommandGroup(
                        new WaitCommand(0.1),
                        new InstantCommand(() -> endEffector.runEndEffectorOuttake()))),
                () -> reefLevel == ReefLevel.L1)
            // after all levels
            .finallyDo(
                () -> {
                  endEffector.stopEndEffector();
                  GamePieceVisualizer.setHasCoral(false);
                }));
  }
}
