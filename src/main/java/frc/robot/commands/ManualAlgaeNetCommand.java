// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;

public class ManualAlgaeNetCommand extends SequentialCommandGroup {
  public ManualAlgaeNetCommand(
      Drive drive, SuperStructure superStructure, EndEffector endEffector) {
    addCommands(
        new InstantCommand(
            () -> LEDSubsystem.getInstance().setStates(LEDSubsystem.LEDStates.SCORING_LINE_UP)),
        new SuperStructureCommand(superStructure, () -> SuperStructureState.SCORE_ALGAE_NET));

    addRequirements(drive, superStructure);
  }
}
