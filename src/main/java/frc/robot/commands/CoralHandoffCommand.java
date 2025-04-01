// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralHandoffCommand extends SequentialCommandGroup {
  /** Creates a new CoralHandoffCommand. */
  public CoralHandoffCommand(
      SuperStructure superStructure, Feeder feeder, EndEffector endEffector) {
    addCommands(
        new SuperStructureCommand(superStructure, () -> SuperStructureState.IDLE),
        // new WaitUntilCommand(
        //     () -> superStructure.getSuperStructureGoal() == SuperStructureState.IDLE),
        new InstantCommand(() -> endEffector.runEndEffectorIntake(), endEffector),
        new SuperStructureCommand(superStructure, () -> SuperStructureState.INTAKE_CORAL),
        new SuperStructureCommand(superStructure, () -> SuperStructureState.IDLE),
        new InstantCommand(() -> feeder.handoffComplete()));
    addRequirements(superStructure);
  }
}
