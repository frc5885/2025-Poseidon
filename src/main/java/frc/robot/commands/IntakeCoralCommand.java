// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.io.beambreak.BeamBreakIOSim;
import frc.robot.subsystems.Collector.Collector;
import frc.robot.util.GamePieces.GamePieceVisualizer;

public class IntakeCoralCommand extends Command {
  private Collector m_collector;

  public IntakeCoralCommand(Collector collector) {
    m_collector = collector;

    addRequirements(m_collector);
  }

  @Override
  public void initialize() {

    m_collector.extendIntake();

    if (m_collector.getBeamBreakIO() instanceof BeamBreakIOSim) {
      ((BeamBreakIOSim) m_collector.getBeamBreakIO()).simulateGamePieceAcquisition();
    }
  }

  @Override
  public void execute() {
    m_collector.runIntake(12.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_collector.retractIntake();
    m_collector.stopIntake();
    GamePieceVisualizer.setHasCoral(true);
  }

  @Override
  public boolean isFinished() {
    return m_collector.isCollected();
  }
}
