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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.vision.heimdall.HeimdallPoseController;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.TunableDouble;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock m_odometryLock = new ReentrantLock();
  private final GyroIO m_gyroIO;
  private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR
  private final HeimdallPoseController m_poseController;
  private final SysIdRoutine m_driveSysId;
  private final SysIdRoutine m_turnSysId;
  private final Alert m_gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private final Consumer<Pose2d> m_resetSimulationPoseCallBack;

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(kModuleTranslations);
  private Rotation2d m_rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] m_lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final SwerveSetpointGenerator m_setpointGenerator;
  private SwerveSetpoint m_previousSetpoint;

  private DoubleSupplier adjustmentBaseFactor =
      TunableDouble.register("Drive/AdjustmentBaseFactor", 0.3);
  @Setter private DoubleSupplier adjustmentFactor = () -> 0.0;

  private Pose2d m_pathPlannerSetpoint = new Pose2d();
  private boolean m_usePPRunVelocity = false;
  private Pose2d m_pathPlannerStartPose = new Pose2d();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      HeimdallPoseController poseController,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    m_gyroIO = gyroIO;
    m_modules[0] = new Module(flModuleIO, 0);
    m_modules[1] = new Module(frModuleIO, 1);
    m_modules[2] = new Module(blModuleIO, 2);
    m_modules[3] = new Module(brModuleIO, 3);
    m_poseController = poseController;
    m_resetSimulationPoseCallBack = resetSimulationPoseCallBack;

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::ppGetPose,
        this::setPose,
        this::getChassisSpeeds,
        this::ppRunVelocity,
        kPPController,
        kPPConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          m_pathPlannerSetpoint = targetPose;
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
    m_setpointGenerator =
        new SwerveSetpointGenerator(kPPConfig, kMaxModuleRotationVelocityRadiansPerSec);
    m_previousSetpoint =
        new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));

    // Configure SysId
    m_driveSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    m_turnSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(12.0),
                null,
                (state) -> Logger.recordOutput("Drive/TurnSysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runTurnCharacterization(voltage.in(Volts)), null, this));

    // Reset gyro
    resetGyro();
  }

  @Override
  public void periodic() {
    m_odometryLock.lock(); // Prevents odometry updates while reading data
    m_gyroIO.updateInputs(m_gyroInputs);
    Logger.processInputs("Drive/Gyro", m_gyroInputs);
    for (var module : m_modules) {
      module.periodic();
    }
    m_odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : m_modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        m_modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = m_modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - m_lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        m_lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (m_gyroInputs.connected) {
        // Use the real gyro angle
        m_rawGyroRotation = m_gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = m_kinematics.toTwist2d(moduleDeltas);
        m_rawGyroRotation = m_rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      m_poseController.updateWithTime(
          sampleTimestamps[i], m_rawGyroRotation, modulePositions, getChassisSpeeds());
    }

    // Update gyro alert
    m_gyroDisconnectedAlert.set(!m_gyroInputs.connected && Constants.kCurrentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // speed adjustment
    double linearMagnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    Rotation2d linearDirection =
        new Rotation2d(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond));
    Translation2d adjustmentVector =
        new Translation2d(
            linearMagnitude * adjustmentBaseFactor.getAsDouble() * adjustmentFactor.getAsDouble(),
            linearDirection);
    speeds = speeds.minus(new ChassisSpeeds(adjustmentVector.getX(), adjustmentVector.getY(), 0.0));

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = m_kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxSpeedMetersPerSec);

    // Alternate setpoint generation using 254's swerve setpoint generator (needs to be tested)
    // m_previousSetpoint = m_setpointGenerator.generateSetpoint(m_previousSetpoint, speeds, 0.02);
    // SwerveModuleState[] setpointStates = m_previousSetpoint.moduleStates();
    // SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);
    // ChassisSpeeds discreteSpeeds = speeds;

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      m_modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive at the desired velocity using the path planner, if the usePPRunVelocity flag is
   * true.
   */
  public void ppRunVelocity(ChassisSpeeds speeds) {
    if (m_usePPRunVelocity) {
      runVelocity(speeds);
    }
  }

  public void setUsePPRunVelocity(boolean usePPRunVelocity) {
    m_usePPRunVelocity = usePPRunVelocity;
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].runCharacterization(output);
    }
  }

  public void runTurnCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].runTurnCharacterization(output);
    }
  }

  /** Spins the modules in place with the specified output */
  public void runTurnOpenLoop(double output) {
    for (int i = 0; i < 4; i++) {
      m_modules[i].runTurnOpenLoop(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = kModuleTranslations[i].getAngle();
    }
    m_kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(m_driveSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(m_driveSysId.dynamic(direction));
  }

  /** Returns a command to run a quasistatic test in the specified direction on the turn motor. */
  public Command turnSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runTurnCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(m_turnSysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction on the turn motor. */
  public Command turnSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runTurnCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(m_turnSysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = m_modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * if {@code doNotFlip} is true, the robot will not flip the path when generating a path to a
   * pose.
   *
   * @param pose
   * @param doNotFlip
   * @return The command to drive to the specified pose.
   */
  public Command getDriveToPoseCommand(Supplier<Pose2d> pose, boolean doNotFlip) {
    return doNotFlip
        ? AutoBuilder.pathfindToPose(pose.get(), kPathConstraintsFast)
        : AutoBuilder.pathfindToPoseFlipped(pose.get(), kPathConstraintsFast);
  }

  public Command getBetterDriveToPoseCommand(
      Supplier<Pose2d> startPose, Supplier<Pose2d> endPose, boolean doNotFlip) {
    m_pathPlannerStartPose = startPose.get();
    Command cmd =
        doNotFlip
            ? AutoBuilder.pathfindToPose(endPose.get(), kPathConstraintsFast)
            : AutoBuilder.pathfindToPoseFlipped(endPose.get(), kPathConstraintsFast);
    // m_pathPlannerStartPose = new Pose2d();
    return cmd;
  }

  public Command getPathFollowCommand(Supplier<Pose2d> target) {
    PathPlannerPath path =
        new PathPlannerPath(
            PathPlannerPath.waypointsFromPoses(
                getPose(),
                target.get().transformBy(new Transform2d(-0.5, 0.0, Rotation2d.kZero)),
                target.get()),
            kPathConstraintsFast,
            null,
            new GoalEndState(0.0, target.get().getRotation()));
    path.preventFlipping = true;
    return AutoBuilder.followPath(path);
  }

  public Command getPathFollowBackOutCommand(Supplier<Pose2d> target) {
    List<PathPoint> pathPoints =
        new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(target.get(), getPose()),
                kPathConstraintsFast,
                null,
                new GoalEndState(0.0, getRotation()))
            .getAllPathPoints();
    Collections.reverse(pathPoints);
    pathPoints.remove(0);

    PathPlannerPath path =
        PathPlannerPath.fromPathPoints(
            pathPoints, kPathConstraintsFast, new GoalEndState(-0.2, target.get().getRotation()));
    path.preventFlipping = true;
    return AutoBuilder.followPath(path);
  }

  /**
   * Build a command to pathfind to a given path, then follow that path.
   *
   * @param goalPath The path to pathfind to, then follow
   * @return A command to pathfind to a given path, then follow the path
   */
  public Command getPathFindFollowCommand(Supplier<PathPlannerPath> goalPath) {
    return AutoBuilder.pathfindThenFollowPath(goalPath.get(), kPathConstraintsFast);
  }

  public Pose2d getPathPlannerSetpoint() {
    return m_pathPlannerSetpoint;
  }

  public void setPathPlannerSetpoint(Pose2d setpoint) {
    m_pathPlannerSetpoint = setpoint;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = m_modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += m_modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return m_poseController.getEstimatedPosition();
  }

  /**
   * Returns the current odometry pose for path planner. Certain use cases set the start pose to
   * some pose in the future rather than the real robot pose.
   */
  public Pose2d ppGetPose() {
    return m_pathPlannerStartPose.equals(new Pose2d()) ? getPose() : m_pathPlannerStartPose;
  }
  /**
   * Resets the current odometry pose for path planner to use the real robot pose. Certain use cases
   * set the start pose to some pose in the future rather than the real robot pose.
   */
  public void resetPathPlannerGetPose() {
    m_pathPlannerStartPose = new Pose2d();
    m_pathPlannerSetpoint = new Pose2d();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    m_resetSimulationPoseCallBack.accept(pose);
    m_poseController.resetPosition(m_rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs,
      double averageTagDistance) {
    m_poseController.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs, averageTagDistance);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return kMaxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return kMaxAngularSpeedRadiansPerSec;
  }

  /**
   * Returns the current rotation velocity of the swerve module with the given index in rad per sec.
   * Only used for configuration of swerve setpoint generator
   */
  public double getModuleRotationVelocityRadPerSec(int moduleIndex) {
    return m_modules[moduleIndex].getModuleRotationVelocityRadPerSec();
  }

  /** Reset the gyro */
  public void resetGyro() {
    m_gyroIO.resetGyro();
  }

  public HeimdallPoseController getHeimdall() {
    return m_poseController;
  }
}
