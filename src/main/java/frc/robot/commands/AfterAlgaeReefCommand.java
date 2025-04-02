// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CalculateAlgaeStateUtil;
import java.util.Set;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AfterAlgaeReefCommand extends SequentialCommandGroup {
  /** Creates a new AfterAlgaeReefCommand. */
  public AfterAlgaeReefCommand(
      Drive drive, SuperStructure superStructure, EndEffector endEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(
            () -> {
              LEDSubsystem.getInstance().setStates(LEDStates.RESETTING_SUPERSTRUCTURE);
              endEffector.holdAlgae();
            }),
        new SuperStructureCommand(
            superStructure,
            () ->
                CalculateAlgaeStateUtil.calculateAfterIntakeState(
                    superStructure.getSuperStructureGoal())),
        new DeferredCommand(
            () -> new ResetSuperStructureCommand(drive, superStructure, true),
            Set.of(superStructure)),
        new InstantCommand(() -> LEDSubsystem.getInstance().setStates(LEDStates.IDLE)));

    addRequirements(superStructure);
  }
}
