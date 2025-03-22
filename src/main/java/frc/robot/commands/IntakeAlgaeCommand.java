// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDS.LEDSubsystem;

/** need a .until() at the end */
public class IntakeAlgaeCommand extends Command {
  private EndEffector m_endEffector;

  public IntakeAlgaeCommand(EndEffector endEffector) {
    m_endEffector = endEffector;

    addRequirements(m_endEffector);
  }

  @Override
  public void initialize() {
    LEDSubsystem.getInstance().setStates(LEDSubsystem.LEDStates.INTAKE_RUNNING);
  }

  @Override
  public void execute() {
    m_endEffector.runEndEffectorIntake();
  }

  @Override
  public void end(boolean interrupted) {
    m_endEffector.holdAlgae();
    // m_endEffector.stopEndEffector();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
