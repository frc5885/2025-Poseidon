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

import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import org.littletonrobotics.junction.Logger
import kotlin.math.sign

class Module(private val m_io: ModuleIO, private val m_index: Int) {
    private val m_inputs = ModuleIOInputsAutoLogged()

    private val m_drivePlant: LinearSystem<N1, N1, N1>
    private val m_driveFF: LinearPlantInversionFeedforward<N1, N1, N1>
    private val m_driveRegulator: LinearQuadraticRegulator<N1, N1, N1>
    private var m_driveFFVolts = 0.0
    private val m_driveController: PIDController
    private val m_turnController: PIDController
    private var isClosedLoop = false
 
    private val m_driveDisconnectedAlert: Alert
    private val m_turnDisconnectedAlert: Alert

    /** Returns the module positions received this cycle.  */
    var odometryPositions: List<SwerveModulePosition?> = listOf()
        private set

    init {
        SmartDashboard.putBoolean("DriveStateSpace", false)
        m_drivePlant = LinearSystemId.identifyVelocitySystem(DriveConstants.kDriveKv, DriveConstants.kDriveKa)
        m_driveFF = LinearPlantInversionFeedforward(m_drivePlant, 0.02)
        m_driveRegulator =
            LinearQuadraticRegulator(
                m_drivePlant, VecBuilder.fill(0.8), VecBuilder.fill(12.0), 0.02
            )

        if (Constants.kCurrentMode == Constants.Mode.REAL) {
            m_driveRegulator.latencyCompensate(m_drivePlant, 0.02, DriveConstants.kModuleLatencyCompensationMs)
        }

        m_driveController = PIDController(DriveConstants.kDriveKp, 0.0, DriveConstants.kDriveKd)
        m_turnController = PIDController(DriveConstants.kTurnKp, 0.0, DriveConstants.kTurnKd)
        m_turnController.enableContinuousInput(-Math.PI, Math.PI)

        m_driveDisconnectedAlert =
            Alert(
                "Disconnected drive motor on module $m_index.",
                AlertType.kError
            )
        m_turnDisconnectedAlert =
            Alert(
                "Disconnected turn motor on module $m_index.",
                AlertType.kError
            )
    }

    fun periodic() {
        m_io.updateInputs(m_inputs)
        Logger.processInputs("Drive/Module$m_index", m_inputs)

        if (SmartDashboard.getBoolean("DriveStateSpace", false)) {
            if (isClosedLoop) {
                val driveSetpoint = m_driveController.setpoint
                val nextDriveR = VecBuilder.fill(driveSetpoint)

                m_io.setDriveOpenLoop(
                    m_driveRegulator
                        .calculate(VecBuilder.fill(m_inputs.driveVelocityRadPerSec), nextDriveR)
                        .plus(
                            m_driveFF.calculate(nextDriveR)
                                .plus(DriveConstants.kDriveKs * sign(driveSetpoint))
                        )[0, 0]
                )

                m_io.setTurnOpenLoop(m_turnController.calculate(m_inputs.turnPosition.radians))
            } else {
                m_driveController.reset()
                m_turnController.reset()
            }
        } else {
            if (isClosedLoop) {
                m_io.setDriveOpenLoop(
                    m_driveFFVolts + m_driveController.calculate(m_inputs.driveVelocityRadPerSec)
                )
                m_io.setTurnOpenLoop(m_turnController.calculate(m_inputs.turnPosition.radians))
            } else {
                m_driveController.reset()
                m_turnController.reset()
            }
        }

        // Calculate positions for odometry
        odometryPositions = m_inputs.odometryDrivePositionsRad
            .zip(m_inputs.odometryTurnPositions) { driveRad, angle ->
                SwerveModulePosition(driveRad * DriveConstants.kWheelRadiusMeters, angle)
            }

        // Update alerts
        m_driveDisconnectedAlert.set(!m_inputs.driveConnected)
        m_turnDisconnectedAlert.set(!m_inputs.turnConnected)
    }

    /** Runs the module with the specified setpoint state. Mutates the state to optimize it.  */
    fun runSetpoint(state: SwerveModuleState) {
        // Optimize velocity setpoint
        state.optimize(angle)
        state.cosineScale(m_inputs.turnPosition)

        isClosedLoop = true
        val velocityRadPerSec = state.speedMetersPerSecond / DriveConstants.kWheelRadiusMeters
        m_driveFFVolts = DriveConstants.kDriveKs * sign(velocityRadPerSec) + DriveConstants.kDriveKv * velocityRadPerSec
        m_driveController.setpoint = velocityRadPerSec
        m_turnController.setpoint = state.angle.plus(m_io.zeroRotation).radians
    }

    /** Runs the module with the specified output while controlling to zero degrees.  */
    fun runCharacterization(output: Double) {
        isClosedLoop = false
        m_io.setDriveOpenLoop(output)
        m_turnController.setpoint = m_io.zeroRotation.radians
        m_io.setTurnOpenLoop(m_turnController.calculate(m_inputs.turnPosition.getRadians()))
    }

    fun runTurnCharacterization(output: Double) {
        isClosedLoop = false
        m_io.setTurnOpenLoop(output)
    }

    /** Runs the turn motor with the specified output.  */
    fun runTurnOpenLoop(output: Double) {
        isClosedLoop = false
        m_io.setTurnOpenLoop(output)
    }

    /** Disables all outputs to motors.  */
    fun stop() {
        m_io.setDriveOpenLoop(0.0)
        m_io.setTurnOpenLoop(0.0)
    }

    val angle: Rotation2d
        /** Returns the current turn angle of the module.  */
        get() = m_inputs.turnPosition

    val positionMeters: Double
        /** Returns the current drive position of the module in meters.  */
        get() = m_inputs.drivePositionRad * DriveConstants.kWheelRadiusMeters

    val velocityMetersPerSec: Double
        /** Returns the current drive velocity of the module in meters per second.  */
        get() = m_inputs.driveVelocityRadPerSec * DriveConstants.kWheelRadiusMeters

    val moduleRotationVelocityRadPerSec: Double
        /**
         * Returns the current rotation velocity of the module in rad per sec. Only used for configuration
         * of swerve setpoint generator.
         */
        get() = m_inputs.turnVelocityRadPerSec

    val position: SwerveModulePosition
        /** Returns the module position (turn angle and drive position).  */
        get() = SwerveModulePosition(positionMeters, angle)

    val state: SwerveModuleState
        /** Returns the module state (turn angle and drive velocity).  */
        get() = SwerveModuleState(velocityMetersPerSec, angle)

    val odometryTimestamps: List<Double>
        /** Returns the timestamps of the samples received this cycle.  */
        get() = m_inputs.odometryTimestamps

    val wheelRadiusCharacterizationPosition: Double
        /** Returns the module position in radians.  */
        get() = m_inputs.drivePositionRad

    val fFCharacterizationVelocity: Double
        /** Returns the module velocity in rad/sec.  */
        get() = m_inputs.driveVelocityRadPerSec
}
