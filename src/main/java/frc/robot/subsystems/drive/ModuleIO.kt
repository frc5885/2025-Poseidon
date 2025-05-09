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

import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

interface ModuleIO {
    @AutoLog
    open class ModuleIOInputs {
        var driveConnected = false
        var drivePositionRad = 0.0
        var driveVelocityRadPerSec = 0.0
        var driveAppliedVolts = 0.0
        var driveCurrentAmps = 0.0
        var turnConnected = false
        var turnPosition = Rotation2d()
        var turnAbsolutePosition = Rotation2d()
        var turnVelocityRadPerSec = 0.0
        var turnAppliedVolts = 0.0
        var turnCurrentAmps = 0.0
        var odometryTimestamps = emptyList<Double>()
        var odometryDrivePositionsRad = emptyList<Double>()
        var odometryTurnPositions = emptyList<Rotation2d>()
    }

    val zeroRotation: Rotation2d
        get() = Rotation2d()

    /** Updates the set of loggable inputs.  */
    fun updateInputs(inputs: ModuleIOInputs) {}

    /** Run the drive motor at the specified open loop value.  */
    fun setDriveOpenLoop(output: Double) {}

    /** Run the turn motor at the specified open loop value.  */
    fun setTurnOpenLoop(output: Double) {}
}
