// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.io.beambreak.BeamBreakIOSim;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;
import frc.robot.util.GamePieces.GamePieceVisualizer;

public class ScoreCoralCommand extends Command {
  private final EndEffector m_endEffector;
  /**
   * A command that scores a coral. Ends when the coral exits the end effector and un-triggers the
   * beambreak.
   */
  public ScoreCoralCommand(EndEffector endEffector) {
    m_endEffector = endEffector;

    addRequirements(m_endEffector);
  }

  @Override
  public void initialize() {
    // Simulate a coral being scored
    if (m_endEffector.getCoralBeamBreakIO() instanceof BeamBreakIOSim) {
      ((BeamBreakIOSim) m_endEffector.getCoralBeamBreakIO()).simulateGamePieceOuttake(0.5);
    }
    LEDSubsystem.getInstance().setStates(LEDStates.SCORED);
  }

  @Override
  public void execute() {
    m_endEffector.runCoralEjector(12.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_endEffector.stopCoralEjector();

    // Don't simulate a successful outtake if the command was interrupted
    if (interrupted) {
      if (m_endEffector.getCoralBeamBreakIO() instanceof BeamBreakIOSim) {
        ((BeamBreakIOSim) m_endEffector.getCoralBeamBreakIO()).cancelSimulatedGamePieceChange();
      }
    } else {
      if (Constants.kCurrentMode != Mode.REAL) {
        GamePieceVisualizer.scoreCoral().schedule();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return !m_endEffector.isCoralHeld();
  }
}
