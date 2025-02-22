package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;

public class ResetSuperStructureCommand extends SequentialCommandGroup {
  public ResetSuperStructureCommand(Drive drive, SuperStructure superStructure) {
    addCommands(
        new InstantCommand(
            () -> LEDSubsystem.getInstance().setStates(LEDStates.RESETTING_SUPERSTRUCTURE)),
        new WaitUntilFarFromCommand(drive::getPose, 0.5),
        new SuperStructureCommand(superStructure, () -> SuperStructureState.INTAKE_CORAL),
        new InstantCommand(() -> LEDSubsystem.getInstance().setStates(LEDStates.IDLE)));
  }
}
