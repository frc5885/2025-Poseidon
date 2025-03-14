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

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.SparkUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
  private final SwerveModuleSimulation m_moduleSimulation;
  private final SimulatedMotorController.GenericMotorController m_driveMotor;
  private final SimulatedMotorController.GenericMotorController m_turnMotor;

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
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePositionRad = m_moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec =
        m_moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    inputs.driveAppliedVolts = m_moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);
    inputs.driveCurrentAmps = Math.abs(m_moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

    // Update turn inputs
    inputs.turnConnected = true;
    inputs.turnPosition = m_moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnAbsolutePosition = m_moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnVelocityRadPerSec =
        m_moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = m_moduleSimulation.getSteerMotorAppliedVoltage().in(Volts);
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
    m_driveMotor.requestVoltage(Volts.of(MathUtil.clamp(output, -12.0, 12.0)));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    m_turnMotor.requestVoltage(Volts.of(MathUtil.clamp(output, -12.0, 12.0)));
  }
}
