// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import java.util.function.Supplier;

public class SuperStructureCommand extends Command {
  private final SuperStructure m_superStructure;
  private final Supplier<SuperStructureState> m_stateSupplier;
  private Command m_command;

  /**
   * A command that sets the superstructure to a given state. Ends when the superstructure has
   * reached the desired state.
   */
  public SuperStructureCommand(
      SuperStructure superStructure, Supplier<SuperStructureState> stateSupplier) {
    m_superStructure = superStructure;
    m_stateSupplier = stateSupplier;

    addRequirements(superStructure);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SuperStructureState newState = m_stateSupplier.get();
    if (LEDSubsystem.getInstance().isAlgaeHeld() && newState == SuperStructureState.IDLE_CORAL) {
      newState = SuperStructureState.IDLE_ALGAE;
    }
    m_command = m_superStructure.setSuperStructureGoal(newState);
    m_command.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_superStructure.isFinalGoalAchieved();
  }
}
