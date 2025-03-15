// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Collector;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.beambreak.BeamBreakIO;
import frc.robot.subsystems.Collector.Feeder.Feeder;
import frc.robot.subsystems.Collector.Feeder.FeederIO;
import frc.robot.subsystems.Collector.Intake.Intake;
import frc.robot.subsystems.Collector.Intake.IntakeIO;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;

public class Collector extends SubsystemBase {
  private final Intake m_intake;
  private final Feeder m_feeder;

  /** Creates a new Collector. */
  public Collector(IntakeIO intakeIO, FeederIO feederIO, BeamBreakIO beamBreakIO) {
    m_intake = new Intake(intakeIO);
    m_feeder = new Feeder(feederIO, beamBreakIO);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_intake.periodic();
    m_feeder.periodic();

    visualizationUpdate();
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

  public boolean isIntakeExtended() {
    return m_intake.isExtended();
  }

  public void extendIntake() {
    m_intake.extend();
  }

  public void retractIntake() {
    m_intake.retract();
  }

  public IntakeSimulation getMapleIntakeSim() {
    return m_intake.getMapleIntakeSim();
  }

  private void visualizationUpdate() {
    // Log pose 3d
    Logger.recordOutput(
        "SuperStructure/Mechanism3d/4-Intake",
        isIntakeExtended()
            ? new Pose3d(-0.2, 0.0, 0.18, new Rotation3d(0, 0, 0))
            : new Pose3d(-0.2, 0.0, 0.18, new Rotation3d(0, Units.degreesToRadians(90), 0)));
  }
}
