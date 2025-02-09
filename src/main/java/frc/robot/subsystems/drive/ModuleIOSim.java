// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim m_driveSim;
  private final DCMotorSim m_turnSim;

  private boolean m_driveClosedLoop = false;
  private boolean m_turnClosedLoop = false;
  private PIDController m_driveController = new PIDController(kDriveSimP, 0, kDriveSimD);
  private PIDController m_turnController = new PIDController(kTurnSimP, 0, kTurnSimD);
  private double m_driveFFVolts = 0.0;
  private double m_driveAppliedVolts = 0.0;
  private double m_turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    // Create drive and turn sim models
    m_driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kDriveGearbox, 0.025, kDriveMotorReduction),
            kDriveGearbox);
    m_turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(kTurnGearbox, 0.004, kTurnMotorReduction),
            kTurnGearbox);

    // Enable wrapping for turn PID
    m_turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (m_driveClosedLoop) {
      m_driveAppliedVolts =
          m_driveFFVolts + m_driveController.calculate(m_driveSim.getAngularVelocityRadPerSec());
    } else {
      m_driveController.reset();
    }
    if (m_turnClosedLoop) {
      m_turnAppliedVolts = m_turnController.calculate(m_turnSim.getAngularPositionRad());
    } else {
      m_turnController.reset();
    }

    // Update simulation state
    m_driveSim.setInputVoltage(MathUtil.clamp(m_driveAppliedVolts, -12.0, 12.0));
    m_turnSim.setInputVoltage(MathUtil.clamp(m_turnAppliedVolts, -12.0, 12.0));
    m_driveSim.update(0.02);
    m_turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = m_driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = m_driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = m_driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(m_driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(m_turnSim.getAngularPositionRad());
    inputs.turnAbsolutePosition = new Rotation2d(m_turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = m_turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = m_turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(m_turnSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    m_driveClosedLoop = false;
    m_driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    m_turnClosedLoop = false;
    m_turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    m_driveClosedLoop = true;
    m_driveFFVolts = kDriveSimKs * Math.signum(velocityRadPerSec) + kDriveSimKv * velocityRadPerSec;
    m_driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    m_turnClosedLoop = true;
    m_turnController.setSetpoint(rotation.getRadians());
  }
}
