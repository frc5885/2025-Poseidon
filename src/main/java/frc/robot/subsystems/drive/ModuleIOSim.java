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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SparkUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation m_moduleSimulation;
  private final SimulatedMotorController.GenericMotorController m_driveMotor;
  private final SimulatedMotorController.GenericMotorController m_turnMotor;

  private boolean m_driveClosedLoop = false;
  private boolean m_turnClosedLoop = false;
  private PIDController m_driveController = new PIDController(kDriveSimP, 0, kDriveSimD);
  private PIDController m_turnController = new PIDController(kTurnSimP, 0, kTurnSimD);
  private double m_driveFFVolts = 0.0;
  private double m_driveAppliedVolts = 0.0;
  private double m_turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    // Create drive and turn sim models
    m_moduleSimulation = moduleSimulation;
    m_driveMotor =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(kDriveMotorCurrentLimit));
    m_turnMotor =
        moduleSimulation
            .useGenericControllerForSteer()
            .withCurrentLimit(Amps.of(kTurnMotorCurrentLimit));

    // Enable wrapping for turn PID
    m_turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (m_driveClosedLoop) {
      m_driveAppliedVolts =
          m_driveFFVolts
              + m_driveController.calculate(
                  m_moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
    } else {
      m_driveController.reset();
    }
    if (m_turnClosedLoop) {
      m_turnAppliedVolts =
          m_turnController.calculate(m_moduleSimulation.getSteerAbsoluteFacing().getRadians());
    } else {
      m_turnController.reset();
    }

    // Update simulation state
    m_driveMotor.requestVoltage(Volts.of(m_driveAppliedVolts));
    m_turnMotor.requestVoltage(Volts.of(m_turnAppliedVolts));

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = m_moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec =
        m_moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = m_driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(m_moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = m_moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnAbsolutePosition = m_moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec =
        m_moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = m_turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(m_moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps();
    inputs.odometryDrivePositionsRad =
        Arrays.stream(m_moduleSimulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();
    inputs.odometryTurnPositions = m_moduleSimulation.getCachedSteerAbsolutePositions();
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
