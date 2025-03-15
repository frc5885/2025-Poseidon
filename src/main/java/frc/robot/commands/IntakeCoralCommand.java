// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Feeder.FeederConstants.FeederState;
import frc.robot.subsystems.LEDS.LEDSubsystem;

public class IntakeCoralCommand extends Command {
  private Feeder m_feeder;
  private EndEffector m_endEffector;

  /**
   * A command that intakes a coral (extends the intake, runs the intake, and retracts the intake).
   * Ends when the coral is collected.
   */
  public IntakeCoralCommand(Feeder feeder, EndEffector endEffector) {
    m_feeder = feeder;
    m_endEffector = endEffector;

    addRequirements(m_feeder);
  }

  @Override
  public void initialize() {
    m_feeder.setFeederState(FeederState.FEEDING);
    LEDSubsystem.getInstance().setStates(LEDSubsystem.LEDStates.INTAKE_RUNNING);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

    // Don't simulate a successful intake if the command was interrupted
    // if (interrupted) {
    //   if (m_endEffector.getCoralBeamBreakIO() instanceof BeamBreakIOSim) {
    //     ((BeamBreakIOSim) m_endEffector.getCoralBeamBreakIO()).cancelSimulatedGamePieceChange();
    //   }
    // LEDSubsystem.getInstance().setStates(LEDSubsystem.LEDStates.IDLE);
    // } else {
    LEDSubsystem.getInstance().setStates(LEDSubsystem.LEDStates.HOLDING_PIECE);
    // }
  }

  @Override
  public boolean isFinished() {
    return m_endEffector.isCoralHeld();
  }
}
