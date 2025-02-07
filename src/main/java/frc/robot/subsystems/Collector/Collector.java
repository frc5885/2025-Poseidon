// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Collector.intake.Intake;
import frc.robot.subsystems.Collector.intake.IntakeIO;

public class Collector extends SubsystemBase {
  private final Intake m_intake;

  /** Creates a new Collector. */
  public Collector(IntakeIO io) {
    m_intake = new Intake(io);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_intake.periodic();
  }

  public void runIntake(double volts) {
    m_intake.runIntake(volts);
  }

  public void stopIntake() {
    m_intake.stop();
  }
}
