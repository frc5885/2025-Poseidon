// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Collector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Collector.Feeder.Feeder;
import frc.robot.subsystems.Collector.Feeder.FeederIO;
import frc.robot.subsystems.Collector.Intake.Intake;
import frc.robot.subsystems.Collector.Intake.IntakeIO;

public class Collector extends SubsystemBase {
  private final Intake m_intake;
  private final Feeder m_feeder;

  /** Creates a new Collector. */
  public Collector(IntakeIO intakeIO, FeederIO feederIO) {
    m_intake = new Intake(intakeIO);
    m_feeder = new Feeder(feederIO);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_intake.periodic();
    m_feeder.periodic();
  }

  public void runIntake(double volts) {
    m_intake.runIntake(volts);
  }

  public void stopIntake() {
    m_intake.stop();
  }

  public void runFeeder(double volts) {
    m_feeder.runFeeder(volts);
  }

  public void stopFeeder() {
    m_feeder.stop();
  }
}
