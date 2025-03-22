// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;

public class ScoreAlgaeCommand extends Command {
  private EndEffector m_endEffector;

  public ScoreAlgaeCommand(EndEffector endEffector) {
    m_endEffector = endEffector;

    addRequirements(m_endEffector);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_endEffector.runEndEffectorOuttake();
  }

  @Override
  public void end(boolean interrupted) {
    m_endEffector.stopEndEffector();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
