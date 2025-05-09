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

import com.studica.frc.AHRS
import com.studica.frc.AHRS.NavXComType
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs
import java.util.*

/** IO implementation for NavX.  */
class GyroIONavX : GyroIO {
    private val m_navX = AHRS(NavXComType.kUSB1, DriveConstants.kOdometryFrequency.toInt().toByte().toInt())
    private val m_yawPositionQueue: Queue<Double> = SparkOdometryThread.getInstance().registerSignal { m_navX.angle }
    private val m_yawTimestampQueue: Queue<Double> =
        SparkOdometryThread.getInstance().makeTimestampQueue()

    override fun updateInputs(inputs: GyroIOInputs) {
        inputs.connected = m_navX.isConnected
        inputs.yawPosition = Rotation2d.fromDegrees(-m_navX.angle)
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(-m_navX.rawGyroZ.toDouble())

        inputs.odometryYawTimestamps = m_yawTimestampQueue.toList()
        inputs.odometryYawPositions = m_yawPositionQueue.map { it -> Rotation2d.fromDegrees(-it) }
        m_yawTimestampQueue.clear()
        m_yawPositionQueue.clear()
    }

    override fun resetGyro() {
        m_navX.reset()
    }
}
