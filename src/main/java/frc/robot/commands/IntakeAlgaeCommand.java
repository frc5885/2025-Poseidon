// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.io.beambreak.BeamBreakIOSim;
import frc.robot.subsystems.EndEffector.EndEffector;

public class IntakeAlgaeCommand extends Command {
  private EndEffector m_endEffector;

  public IntakeAlgaeCommand(EndEffector endEffector) {
    m_endEffector = endEffector;

    addRequirements(m_endEffector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_endEffector.runAlgaeClaw(12.0);

    // Simulate an algae being taken in
    if (m_endEffector.getBeamBreakIO() instanceof BeamBreakIOSim) {
      ((BeamBreakIOSim) m_endEffector.getBeamBreakIO()).simulateGamePieceIntake(0.5);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_endEffector.stopAlgaeClaw();

    // Don't simulate a successful intake if the command was interrupted
    if (interrupted) {
      if (m_endEffector.getBeamBreakIO() instanceof BeamBreakIOSim) {
        ((BeamBreakIOSim) m_endEffector.getBeamBreakIO()).cancelSimulatedGamePieceChange();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return m_endEffector.isAlgaeHeld();
  }
}
