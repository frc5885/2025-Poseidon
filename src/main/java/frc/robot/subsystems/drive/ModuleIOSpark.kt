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

import com.revrobotics.RelativeEncoder
import com.revrobotics.spark.SparkAnalogSensor
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkBase.PersistMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs
import frc.robot.util.SparkUtil
import java.util.*
import java.util.function.DoubleSupplier

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
class ModuleIOSpark(module: Int) : ModuleIO {
    override val zeroRotation: Rotation2d = when (module) {
        0 -> DriveConstants.kFrontLeftZeroRotation
        1 -> DriveConstants.kFrontRightZeroRotation
        2 -> DriveConstants.kBackLeftZeroRotation
        3 -> DriveConstants.kBackRightZeroRotation
        else -> Rotation2d()
    }

    // Hardware objects
    private val m_driveSpark = SparkMax(
        when (module) {
            0 -> DriveConstants.kFrontLeftDriveCanId
            1 -> DriveConstants.kFrontRightDriveCanId
            2 -> DriveConstants.kBackLeftDriveCanId
            3 -> DriveConstants.kBackRightDriveCanId
            else -> 0
        },
        SparkLowLevel.MotorType.kBrushless
    )
    private val m_turnSpark = SparkMax(
        when (module) {
            0 -> DriveConstants.kFrontLeftTurnCanId
            1 -> DriveConstants.kFrontRightTurnCanId
            2 -> DriveConstants.kBackLeftTurnCanId
            3 -> DriveConstants.kBackRightTurnCanId
            else -> 0
        },
        SparkLowLevel.MotorType.kBrushless
    )
    private val m_driveEncoder = m_driveSpark.encoder
    private val m_turnEncoder = m_turnSpark.encoder
    private val m_turnAbsoluteEncoder = m_turnSpark.analog

    // Queue inputs from odometry thread
    private val m_timestampQueue: Queue<Double>
    private val m_drivePositionQueue: Queue<Double>
    private val m_turnPositionQueue: Queue<Double>

    // Connection debouncers
    private val m_driveConnectedDebounce = Debouncer(0.5)
    private val m_turnConnectedDebounce = Debouncer(0.5)

    init {
        // Configure drive motor
        val driveConfig = SparkMaxConfig()
        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveConstants.kDriveMotorCurrentLimit)
            .voltageCompensation(12.0)
        driveConfig
            .encoder
            .positionConversionFactor(DriveConstants.kDriveEncoderPositionFactor)
            .velocityConversionFactor(DriveConstants.kDriveEncoderVelocityFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2)
        driveConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((1000.0 / DriveConstants.kOdometryFrequency).toInt())
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20)
        SparkUtil.tryUntilOk(
            m_driveSpark,
            5
        ) {
            m_driveSpark.configure(
                driveConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
            )
        }
        SparkUtil.tryUntilOk(m_driveSpark, 5) { m_driveEncoder.setPosition(0.0) }

        // Configure turn motor
        val turnConfig = SparkMaxConfig()
        turnConfig
            .inverted(DriveConstants.kTurnInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(DriveConstants.kTurnMotorCurrentLimit)
            .voltageCompensation(12.0)
        turnConfig
            .encoder
            .positionConversionFactor(DriveConstants.kTurnEncoderPositionFactor)
            .velocityConversionFactor(DriveConstants.kTurnEncoderVelocityFactor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2)
        turnConfig
            .analogSensor
            .inverted(DriveConstants.kTurnEncoderInverted)
            .positionConversionFactor(DriveConstants.kTurnAbsoluteEncoderPositionFactor)
            .velocityConversionFactor(DriveConstants.kTurnAbsoluteEncoderVelocityFactor)
        turnConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((1000.0 / DriveConstants.kOdometryFrequency).toInt())
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20)
        SparkUtil.tryUntilOk(
            m_turnSpark,
            5
        ) {
            m_turnSpark.configure(
                turnConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
            )
        }
        // Reset neo relative encoder using absolute encoder position
        SparkUtil.tryUntilOk(
            m_turnSpark, 5
        ) { m_turnEncoder.setPosition(m_turnAbsoluteEncoder.position) }

        // Create odometry queues
        m_timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue()
        m_drivePositionQueue =
            SparkOdometryThread.getInstance().registerSignal(m_driveSpark) { m_driveEncoder.position }
        m_turnPositionQueue =
            SparkOdometryThread.getInstance().registerSignal(m_turnSpark) { m_turnEncoder.position }
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        // Update drive inputs
        SparkUtil.sparkStickyFault = false
        SparkUtil.ifOk(
            m_driveSpark,
            { m_driveEncoder.position },
            { value: Double -> inputs.drivePositionRad = value })
        SparkUtil.ifOk(
            m_driveSpark,
            { m_driveEncoder.velocity },
            { value: Double -> inputs.driveVelocityRadPerSec = value })
        SparkUtil.ifOk(
            m_driveSpark,
            arrayOf(DoubleSupplier { m_driveSpark.appliedOutput }, DoubleSupplier { m_driveSpark.busVoltage })
        ) { values: DoubleArray -> inputs.driveAppliedVolts = values[0] * values[1] }
        SparkUtil.ifOk(
            m_driveSpark,
            { m_driveSpark.outputCurrent },
            { value: Double -> inputs.driveCurrentAmps = value })
        inputs.driveConnected = m_driveConnectedDebounce.calculate(!SparkUtil.sparkStickyFault)

        // Update turn inputs
        SparkUtil.sparkStickyFault = false
        SparkUtil.ifOk(
            m_turnSpark,
            { m_turnEncoder.position },
            { value: Double -> inputs.turnPosition = Rotation2d(value).minus(zeroRotation) })
        SparkUtil.ifOk(
            m_turnSpark,
            { m_turnAbsoluteEncoder.position },
            { value: Double -> inputs.turnAbsolutePosition = Rotation2d(value) })
        SparkUtil.ifOk(
            m_turnSpark,
            { m_turnEncoder.velocity },
            { value: Double -> inputs.turnVelocityRadPerSec = value })
        SparkUtil.ifOk(
            m_turnSpark,
            arrayOf(DoubleSupplier { m_turnSpark.appliedOutput }, DoubleSupplier { m_turnSpark.busVoltage })
        ) { values: DoubleArray -> inputs.turnAppliedVolts = values[0] * values[1] }
        SparkUtil.ifOk(
            m_turnSpark,
            { m_turnSpark.outputCurrent },
            { value: Double -> inputs.turnCurrentAmps = value })
        inputs.turnConnected = m_turnConnectedDebounce.calculate(!SparkUtil.sparkStickyFault)

        // Update odometry inputs
        inputs.odometryTimestamps = m_timestampQueue.toList()
        inputs.odometryDrivePositionsRad = m_drivePositionQueue.toList()
        inputs.odometryTurnPositions = m_turnPositionQueue.map { Rotation2d(it) - zeroRotation }
        m_timestampQueue.clear()
        m_drivePositionQueue.clear()
        m_turnPositionQueue.clear()
    }

    override fun setDriveOpenLoop(output: Double) {
        m_driveSpark.setVoltage(output)
    }

    override fun setTurnOpenLoop(output: Double) {
        m_turnSpark.setVoltage(output)
    }
}
