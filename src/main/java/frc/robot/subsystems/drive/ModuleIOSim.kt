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
package frc.robot.subsystems.drive

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs
import frc.robot.util.SparkUtil
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation
import kotlin.math.abs

/** Physics sim implementation of module IO.  */
class ModuleIOSim(private val m_moduleSimulation: SwerveModuleSimulation) : ModuleIO {
    // Create drive and turn sim models
    private val m_driveMotor = m_moduleSimulation
        .useGenericMotorControllerForDrive()
        .withCurrentLimit(Units.Amps.of(DriveConstants.kDriveMotorCurrentLimit.toDouble()))
    private val m_turnMotor = m_moduleSimulation
        .useGenericControllerForSteer()
        .withCurrentLimit(Units.Amps.of(DriveConstants.kTurnMotorCurrentLimit.toDouble()))

    override fun updateInputs(inputs: ModuleIOInputs) {
        // Update drive inputs
        inputs.driveConnected = true
        inputs.drivePositionRad = m_moduleSimulation.driveWheelFinalPosition.`in`(Units.Radians)
        inputs.driveVelocityRadPerSec =
            m_moduleSimulation.driveWheelFinalSpeed.`in`(Units.RadiansPerSecond)
        inputs.driveAppliedVolts = m_moduleSimulation.driveMotorAppliedVoltage.`in`(Units.Volts)
        inputs.driveCurrentAmps = abs(m_moduleSimulation.driveMotorStatorCurrent.`in`(Units.Amps))

        // Update turn inputs
        inputs.turnConnected = true
        inputs.turnPosition = m_moduleSimulation.steerAbsoluteFacing
        inputs.turnAbsolutePosition = m_moduleSimulation.steerAbsoluteFacing
        inputs.turnVelocityRadPerSec =
            m_moduleSimulation.steerAbsoluteEncoderSpeed.`in`(Units.RadiansPerSecond)
        inputs.turnAppliedVolts = m_moduleSimulation.steerMotorAppliedVoltage.`in`(Units.Volts)
        inputs.turnCurrentAmps = abs(m_moduleSimulation.steerMotorStatorCurrent.`in`(Units.Amps))

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
        inputs.odometryTimestamps = SparkUtil.getSimulationOdometryTimeStamps().toList()
        inputs.odometryDrivePositionsRad = m_moduleSimulation.cachedDriveWheelFinalPositions.map { it.`in`(Units.Radians) }
        inputs.odometryTurnPositions = m_moduleSimulation.cachedSteerAbsolutePositions.toList()
    }

    override fun setDriveOpenLoop(output: Double) {
        m_driveMotor.requestVoltage(Units.Volts.of(MathUtil.clamp(output, -12.0, 12.0)))
    }

    override fun setTurnOpenLoop(output: Double) {
        m_turnMotor.requestVoltage(Units.Volts.of(MathUtil.clamp(output, -12.0, 12.0)))
    }
}
