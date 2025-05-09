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

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.pathfinding.Pathfinding
import com.pathplanner.lib.util.DriveFeedforwards
import com.pathplanner.lib.util.PathPlannerLogging
import com.pathplanner.lib.util.swerve.SwerveSetpoint
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator
import edu.wpi.first.hal.FRCNetComm.tInstances
import edu.wpi.first.hal.FRCNetComm.tResourceType
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.robot.Constants
import frc.robot.subsystems.vision.heimdall.HeimdallPoseController
import frc.robot.util.LocalADStarAK
import frc.robot.util.TunableDouble
import lombok.Setter
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import java.util.*
import java.util.concurrent.locks.Lock
import java.util.concurrent.locks.ReentrantLock
import java.util.function.Consumer
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.atan2
import kotlin.math.hypot

class Drive(
    private val m_gyroIO: GyroIO,
    flModuleIO: ModuleIO,
    frModuleIO: ModuleIO,
    blModuleIO: ModuleIO,
    brModuleIO: ModuleIO,
    poseController: HeimdallPoseController,
    resetSimulationPoseCallBack: Consumer<Pose2d>
) : SubsystemBase() {
    private val m_gyroInputs = GyroIOInputsAutoLogged()
    private val m_modules = arrayOfNulls<Module>(4) // FL, FR, BL, BR
    val heimdall: HeimdallPoseController
    private val m_driveSysId: SysIdRoutine
    private val m_turnSysId: SysIdRoutine
    private val m_gyroDisconnectedAlert = Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError)
    private val m_resetSimulationPoseCallBack: Consumer<Pose2d>

    private val m_kinematics = SwerveDriveKinematics(*DriveConstants.kModuleTranslations)
    private var m_rawGyroRotation = Rotation2d()
    private val m_lastModulePositions =  // For delta tracking
        arrayOf<SwerveModulePosition?>(
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition()
        )

    private val m_setpointGenerator: SwerveSetpointGenerator
    private val m_previousSetpoint: SwerveSetpoint

    private val adjustmentBaseFactor: DoubleSupplier = TunableDouble.register("Drive/AdjustmentBaseFactor", 0.3)

    @Setter
    private val adjustmentFactor = DoubleSupplier { 0.0 }

    var pathPlannerSetpoint = Pose2d()
    private var m_usePPRunVelocity = false
    private var m_pathPlannerStartPose = Pose2d()

    init {
        m_modules[0] = Module(flModuleIO, 0)
        m_modules[1] = Module(frModuleIO, 1)
        m_modules[2] = Module(blModuleIO, 2)
        m_modules[3] = Module(brModuleIO, 3)
        heimdall = poseController
        m_resetSimulationPoseCallBack = resetSimulationPoseCallBack

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit)

        // Start odometry thread
        SparkOdometryThread.getInstance().start()

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            { this.ppGetPose() },
            { pose: Pose2d -> this.pose = pose },
            { this.chassisSpeeds },
            { speeds: ChassisSpeeds -> this.ppRunVelocity(speeds) },
            DriveConstants.kPPController,
            DriveConstants.kPPConfig,
            { DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red })
        Pathfinding.setPathfinder(LocalADStarAK())
        PathPlannerLogging.setLogActivePathCallback { activePath: List<Pose2d> ->
            Logger.recordOutput(
                "Odometry/Trajectory", *activePath.toTypedArray<Pose2d>()
            )
        }
        PathPlannerLogging.setLogTargetPoseCallback { targetPose: Pose2d ->
            pathPlannerSetpoint = targetPose
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
        }
        m_setpointGenerator =
            SwerveSetpointGenerator(DriveConstants.kPPConfig, DriveConstants.kMaxModuleRotationVelocityRadiansPerSec)
        m_previousSetpoint =
            SwerveSetpoint(chassisSpeeds, moduleStates, DriveFeedforwards.zeros(4))

        // Configure SysId
        m_driveSysId =
            SysIdRoutine(
                SysIdRoutine.Config(
                    null,
                    null,
                    null
                ) { state: SysIdRoutineLog.State ->
                    Logger.recordOutput(
                        "Drive/SysIdState",
                        state.toString()
                    )
                },
                Mechanism(
                    { voltage: Voltage -> runCharacterization(voltage.`in`(Units.Volts)) }, null, this
                )
            )
        m_turnSysId =
            SysIdRoutine(
                SysIdRoutine.Config(
                    null,
                    Units.Volts.of(12.0),
                    null
                ) { state: SysIdRoutineLog.State ->
                    Logger.recordOutput(
                        "Drive/TurnSysIdState",
                        state.toString()
                    )
                },
                Mechanism(
                    { voltage: Voltage -> runTurnCharacterization(voltage.`in`(Units.Volts)) }, null, this
                )
            )

        // Reset gyro
        resetGyro()
    }

    override fun periodic() {
        m_odometryLock.lock() // Prevents odometry updates while reading data
        m_gyroIO.updateInputs(m_gyroInputs)
        Logger.processInputs("Drive/Gyro", m_gyroInputs)
        for (module in m_modules) {
            module?.periodic()
        }
        m_odometryLock.unlock()

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (module in m_modules) {
                module?.stop()
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", *arrayOf<SwerveModuleState>())
            Logger.recordOutput("SwerveStates/SetpointsOptimized", *arrayOf<SwerveModuleState>())
        }

        // Update odometry
        val sampleTimestamps: List<Double> =
            m_modules[0]!!.odometryTimestamps // All signals are sampled together
        val sampleCount = sampleTimestamps.size
        for (i in 0..<sampleCount) {
            // Read wheel positions and deltas from each module
            val modulePositions = arrayOfNulls<SwerveModulePosition>(4)
            val moduleDeltas = arrayOfNulls<SwerveModulePosition>(4)
            for (moduleIndex in 0..3) {
                modulePositions[moduleIndex] = m_modules[moduleIndex]!!.odometryPositions[i]
                moduleDeltas[moduleIndex] =
                    SwerveModulePosition(
                        modulePositions[moduleIndex]!!.distanceMeters
                                - m_lastModulePositions[moduleIndex]!!.distanceMeters,
                        modulePositions[moduleIndex]!!.angle
                    )
                m_lastModulePositions[moduleIndex] = modulePositions[moduleIndex]
            }

            // Update gyro angle
            if (m_gyroInputs.connected) {
                // Use the real gyro angle
                m_rawGyroRotation = m_gyroInputs.odometryYawPositions[i]
            } else {
                // Use the angle delta from the kinematics and module deltas
                val twist = m_kinematics.toTwist2d(*moduleDeltas)
                m_rawGyroRotation = m_rawGyroRotation.plus(Rotation2d(twist.dtheta))
            }

            // Apply update
            heimdall.updateWithTime(
                sampleTimestamps[i], m_rawGyroRotation, modulePositions, chassisSpeeds
            )
        }

        // Update gyro alert
        m_gyroDisconnectedAlert.set(!m_gyroInputs.connected && Constants.kCurrentMode != Constants.Mode.SIM)
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    fun runVelocity(speeds: ChassisSpeeds) {
        // speed adjustment
        var speeds = speeds
        val linearMagnitude = hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        val linearDirection =
            Rotation2d(atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond))
        val adjustmentVector =
            Translation2d(
                linearMagnitude * adjustmentBaseFactor.asDouble * adjustmentFactor.asDouble,
                linearDirection
            )
        speeds = speeds.minus(ChassisSpeeds(adjustmentVector.x, adjustmentVector.y, 0.0))

        // Calculate module setpoints
        val discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02)
        val setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxSpeedMetersPerSec)

        // Alternate setpoint generation using 254's swerve setpoint generator (needs to be tested)
        // m_previousSetpoint = m_setpointGenerator.generateSetpoint(m_previousSetpoint, speeds, 0.02);
        // SwerveModuleState[] setpointStates = m_previousSetpoint.moduleStates();
        // SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);
        // ChassisSpeeds discreteSpeeds = speeds;

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", *setpointStates)
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds)

        // Send setpoints to modules
        for (i in 0..3) {
            m_modules[i]!!.runSetpoint(setpointStates[i])
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", *setpointStates)
    }

    /**
     * Runs the drive at the desired velocity using the path planner, if the usePPRunVelocity flag is
     * true.
     */
    fun ppRunVelocity(speeds: ChassisSpeeds) {
        if (m_usePPRunVelocity) {
            runVelocity(speeds)
        }
    }

    fun setUsePPRunVelocity(usePPRunVelocity: Boolean) {
        m_usePPRunVelocity = usePPRunVelocity
    }

    /** Runs the drive in a straight line with the specified drive output.  */
    fun runCharacterization(output: Double) {
        for (i in 0..3) {
            m_modules[i]!!.runCharacterization(output)
        }
    }

    fun runTurnCharacterization(output: Double) {
        for (i in 0..3) {
            m_modules[i]!!.runTurnCharacterization(output)
        }
    }

    /** Spins the modules in place with the specified output  */
    fun runTurnOpenLoop(output: Double) {
        for (i in 0..3) {
            m_modules[i]!!.runTurnOpenLoop(output)
        }
    }

    /** Stops the drive.  */
    fun stop() {
        runVelocity(ChassisSpeeds())
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    fun stopWithX() {
        val headings = arrayOfNulls<Rotation2d>(4)
        for (i in 0..3) {
            headings[i] = DriveConstants.kModuleTranslations[i].angle
        }
        m_kinematics.resetHeadings(*headings)
        stop()
    }

    /** Returns a command to run a quasistatic test in the specified direction.  */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return run { runCharacterization(0.0) }
            .withTimeout(1.0)
            .andThen(m_driveSysId.quasistatic(direction))
    }

    /** Returns a command to run a dynamic test in the specified direction.  */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return run { runCharacterization(0.0) }
            .withTimeout(1.0)
            .andThen(m_driveSysId.dynamic(direction))
    }

    /** Returns a command to run a quasistatic test in the specified direction on the turn motor.  */
    fun turnSysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return run { runTurnCharacterization(0.0) }
            .withTimeout(1.0)
            .andThen(m_turnSysId.quasistatic(direction))
    }

    /** Returns a command to run a dynamic test in the specified direction on the turn motor.  */
    fun turnSysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return run { runTurnCharacterization(0.0) }
            .withTimeout(1.0)
            .andThen(m_turnSysId.dynamic(direction))
    }

    @get:AutoLogOutput(key = "SwerveStates/Measured")
    private val moduleStates: Array<SwerveModuleState?>
        /** Returns the module states (turn angles and drive velocities) for all of the modules.  */
        get() {
            val states = arrayOfNulls<SwerveModuleState>(4)
            for (i in 0..3) {
                states[i] = m_modules[i]!!.state
            }
            return states
        }

    private val modulePositions: Array<SwerveModulePosition?>
        /** Returns the module positions (turn angles and drive positions) for all of the modules.  */
        get() {
            val states = arrayOfNulls<SwerveModulePosition>(4)
            for (i in 0..3) {
                states[i] = m_modules[i]!!.position
            }
            return states
        }

    @get:AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    val chassisSpeeds: ChassisSpeeds
        /** Returns the measured chassis speeds of the robot.  */
        get() = m_kinematics.toChassisSpeeds(*moduleStates)

    /**
     * if `doNotFlip` is true, the robot will not flip the path when generating a path to a
     * pose.
     *
     * @param pose
     * @param doNotFlip
     * @return The command to drive to the specified pose.
     */
    fun getDriveToPoseCommand(pose: Supplier<Pose2d?>, doNotFlip: Boolean): Command {
        return if (doNotFlip)
            AutoBuilder.pathfindToPose(pose.get(), DriveConstants.kPathConstraintsFast)
        else
            AutoBuilder.pathfindToPoseFlipped(pose.get(), DriveConstants.kPathConstraintsFast)
    }

    fun getBetterDriveToPoseCommand(
        startPose: Supplier<Pose2d>, endPose: Supplier<Pose2d?>, doNotFlip: Boolean
    ): Command {
        m_pathPlannerStartPose = startPose.get()
        val cmd =
            if (doNotFlip)
                AutoBuilder.pathfindToPose(endPose.get(), DriveConstants.kPathConstraintsFast)
            else
                AutoBuilder.pathfindToPoseFlipped(endPose.get(), DriveConstants.kPathConstraintsFast)
        // m_pathPlannerStartPose = new Pose2d();
        return cmd
    }

    fun getPathFollowCommand(target: Supplier<Pose2d>): Command {
        val path =
            PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(
                    pose,
                    target.get().transformBy(Transform2d(-0.5, 0.0, Rotation2d.kZero)),
                    target.get()
                ),
                DriveConstants.kPathConstraintsFast,
                null,
                GoalEndState(0.0, target.get().rotation)
            )
        path.preventFlipping = true
        return AutoBuilder.followPath(path)
    }

    fun getPathFollowBackOutCommand(target: Supplier<Pose2d>): Command {
        val pathPoints =
            PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(target.get(), pose),
                DriveConstants.kPathConstraintsFast,
                null,
                GoalEndState(0.0, rotation)
            )
                .allPathPoints
        Collections.reverse(pathPoints)
        pathPoints.removeAt(0)

        val path =
            PathPlannerPath.fromPathPoints(
                pathPoints, DriveConstants.kPathConstraintsFast, GoalEndState(-0.2, target.get().rotation)
            )
        path.preventFlipping = true
        return AutoBuilder.followPath(path)
    }

    /**
     * Build a command to pathfind to a given path, then follow that path.
     *
     * @param goalPath The path to pathfind to, then follow
     * @return A command to pathfind to a given path, then follow the path
     */
    fun getPathFindFollowCommand(goalPath: Supplier<PathPlannerPath?>): Command {
        return AutoBuilder.pathfindThenFollowPath(goalPath.get(), DriveConstants.kPathConstraintsFast)
    }

    val wheelRadiusCharacterizationPositions: DoubleArray
        /** Returns the position of each module in radians.  */
        get() {
            val values = DoubleArray(4)
            for (i in 0..3) {
                values[i] = m_modules[i]!!.wheelRadiusCharacterizationPosition
            }
            return values
        }

    val fFCharacterizationVelocity: Double
        /** Returns the average velocity of the modules in rad/sec.  */
        get() {
            var output = 0.0
            for (i in 0..3) {
                output += m_modules[i]!!.fFCharacterizationVelocity / 4.0
            }
            return output
        }

    @get:AutoLogOutput(key = "Odometry/Robot")
    var pose: Pose2d
        /** Returns the current odometry pose.  */
        get() = heimdall.estimatedPosition
        /** Resets the current odometry pose.  */
        set(pose) {
            m_resetSimulationPoseCallBack.accept(pose)
            heimdall.resetPosition(m_rawGyroRotation, modulePositions, pose)
        }

    /**
     * Returns the current odometry pose for path planner. Certain use cases set the start pose to
     * some pose in the future rather than the real robot pose.
     */
    fun ppGetPose(): Pose2d {
        return if (m_pathPlannerStartPose == Pose2d()) pose else m_pathPlannerStartPose
    }

    /**
     * Resets the current odometry pose for path planner to use the real robot pose. Certain use cases
     * set the start pose to some pose in the future rather than the real robot pose.
     */
    fun resetPathPlannerGetPose() {
        m_pathPlannerStartPose = Pose2d()
        pathPlannerSetpoint = Pose2d()
    }

    val rotation: Rotation2d
        /** Returns the current odometry rotation.  */
        get() = pose.rotation

    /** Adds a new timestamped vision measurement.  */
    fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d?,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3?, N1?>?,
        averageTagDistance: Double
    ) {
        heimdall.addVisionMeasurement(
            visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs, averageTagDistance
        )
    }

    val maxLinearSpeedMetersPerSec: Double
        /** Returns the maximum linear speed in meters per sec.  */
        get() = DriveConstants.kMaxSpeedMetersPerSec

    val maxAngularSpeedRadPerSec: Double
        /** Returns the maximum angular speed in radians per sec.  */
        get() = DriveConstants.kMaxAngularSpeedRadiansPerSec

    /**
     * Returns the current rotation velocity of the swerve module with the given index in rad per sec.
     * Only used for configuration of swerve setpoint generator
     */
    fun getModuleRotationVelocityRadPerSec(moduleIndex: Int): Double {
        return m_modules[moduleIndex]!!.moduleRotationVelocityRadPerSec
    }

    /** Reset the gyro  */
    fun resetGyro() {
        m_gyroIO.resetGyro()
    }

    companion object {
        @JvmField
        val m_odometryLock: Lock = ReentrantLock()
    }
}
