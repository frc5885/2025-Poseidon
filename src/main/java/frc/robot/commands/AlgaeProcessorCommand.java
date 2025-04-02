// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeProcessorCommand extends SequentialCommandGroup {
  /** Creates a new AlgaeProcessorCommand. */
  public AlgaeProcessorCommand(SuperStructure superStructure) {
    addCommands(
        new InstantCommand(() -> LEDSubsystem.getInstance().setStates(LEDStates.SCORING_LINE_UP)));

    Command slowDropArmCommand =
        new Command() {
          @Override
          public void initialize() {
            superStructure.runArmOpenLoop(-4);
          }

          @Override
          public boolean isFinished() {
            return superStructure.getArmPositionRads()
                <= Units.degreesToRadians(ArmGoals.PROCESSOR.setpointDegrees.getAsDouble());
          }
        };

    addCommands(slowDropArmCommand);
    addCommands(
        new InstantCommand(
            () -> superStructure.forceSetCurrentState(SuperStructureState.SCORE_ALGAE_PROCESSOR)),
        new SuperStructureCommand(superStructure, () -> SuperStructureState.SCORE_ALGAE_PROCESSOR));
    addRequirements(superStructure);
  }
}
