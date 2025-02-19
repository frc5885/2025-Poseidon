// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.io.beambreak.BeamBreakIOSim;
import frc.robot.subsystems.Collector.Collector;

public class IntakeCoralCommand extends Command {
  private Collector m_collector;

  /**
   * A command that intakes a coral (extends the intake, runs the intake, and retracts the intake).
   * Ends when the coral is collected.
   */
  public IntakeCoralCommand(Collector collector) {
    m_collector = collector;

    addRequirements(m_collector);
  }

  @Override
  public void initialize() {
    m_collector.extendIntake();
  }

  @Override
  public void execute() {
    m_collector.runIntake(12.0);

    // Simulate a coral being taken in
    if (m_collector.getBeamBreakIO() instanceof BeamBreakIOSim) {
      if (m_collector.getMapleIntakeSim().getGamePiecesAmount() > 0) {
        ((BeamBreakIOSim) m_collector.getBeamBreakIO()).simulateGamePieceIntake(1.0);
        m_collector
            .getMapleIntakeSim()
            .obtainGamePieceFromIntake(); // remove the coral from the intake
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_collector.retractIntake();
    m_collector.stopIntake();

    // Don't simulate a successful intake if the command was interrupted
    if (interrupted) {
      if (m_collector.getBeamBreakIO() instanceof BeamBreakIOSim) {
        ((BeamBreakIOSim) m_collector.getBeamBreakIO()).cancelSimulatedGamePieceChange();
      }
    }
  }

  @Override
  public boolean isFinished() {
    return m_collector.isCollected();
  }
}
