// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.io.beambreak.BeamBreakIOSim;
import frc.robot.subsystems.Collector.Collector;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.util.GamePieces.GamePieceVisualizer;

public class IntakeCoralCommand extends Command {
  private Collector m_collector;
  private EndEffector m_endEffector;

  /**
   * A command that intakes a coral (extends the intake, runs the intake, and retracts the intake).
   * Ends when the coral is collected.
   */
  public IntakeCoralCommand(Collector collector, EndEffector endEffector) {
    m_collector = collector;
    m_endEffector = endEffector;

    addRequirements(m_collector);
  }

  @Override
  public void initialize() {
    m_collector.extendIntake();
    LEDSubsystem.getInstance().setStates(LEDSubsystem.LEDStates.INTAKE_RUNNING);
  }

  @Override
  public void execute() {
    m_collector.runIntake(12.0);
    m_collector.runFeeder(12.0);

    // Simulate a coral being taken in
    if (m_endEffector.getCoralBeamBreakIO() instanceof BeamBreakIOSim) {
      if (m_collector.getMapleIntakeSim().getGamePiecesAmount() > 0) {
        ((BeamBreakIOSim) m_endEffector.getCoralBeamBreakIO()).simulateGamePieceIntake(0.5);
        m_collector
            .getMapleIntakeSim()
            .obtainGamePieceFromIntake(); // remove the coral from the intake
        GamePieceVisualizer.respawnCoral();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_collector.retractIntake();
    m_collector.stopIntake();

    // Don't simulate a successful intake if the command was interrupted
    if (interrupted) {
      if (m_endEffector.getCoralBeamBreakIO() instanceof BeamBreakIOSim) {
        ((BeamBreakIOSim) m_endEffector.getCoralBeamBreakIO()).cancelSimulatedGamePieceChange();
      }
      LEDSubsystem.getInstance().setStates(LEDSubsystem.LEDStates.IDLE);
    } else {
      LEDSubsystem.getInstance().setStates(LEDSubsystem.LEDStates.HOLDING_PIECE);
    }
  }

  @Override
  public boolean isFinished() {
    return m_endEffector.isCoralHeld();
  }
}
