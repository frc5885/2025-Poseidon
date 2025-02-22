// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;

public class ScoreAlgaeProcessor extends SequentialCommandGroup {
  public ScoreAlgaeProcessor(Drive drive, SuperStructure superStructure, EndEffector endEffector) {
    addCommands(
        new InstantCommand(() -> LEDSubsystem.getInstance().setStates(LEDStates.SCORING_LINE_UP)),
        new ParallelCommandGroup(
            new SuperStructureCommand(
                superStructure, () -> SuperStructureState.SCORE_ALGAE_PROCESSOR),
            DriveCommands.preciseChassisAlign(drive, () -> FieldConstants.Processor.centerFace)),
        new ScoreAlgaeCommand(endEffector));
  }
}
