// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Special shoutout to OpenAI's o3-mini-hard

package frc.robot.subsystems.vision.heimdall;

import static frc.robot.subsystems.drive.DriveConstants.moduleTranslations;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.questnav.QuestNav;
import frc.robot.subsystems.vision.questnav.QuestNavIO;
import frc.robot.subsystems.vision.questnav.QuestNavIOReal;
import frc.robot.subsystems.vision.questnav.QuestNavIOSim;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class HeimdallPoseController {
  // -------- Constants / thresholds -------------
  private static final double BUFFER_DURATION = 0.5; // seconds for our buffers

  // When Quest is synced, if its (adjusted) pose diverges too far from odometry, we unsync.
  private static final double MAX_POSE_DIFF = 0.5; // meters+radians (after sync)

  // When not synced, we wait until the offset between Quest and odometry is steady
  // (i.e. its derivative is below this threshold) before syncing.
  private static final double OFFSET_DERIVATIVE_THRESHOLD = 0.02; // (m + rad)/s

  // We consider the robot “stationary” if the normalized chassis speed is below this.
  private static final double STATIONARY_VELOCITY_THRESHOLD = 0.05;
  // And we require the robot to be stationary for at least this long before re-syncing.
  private static final double MIN_TIME_STATIONARY_FOR_SYNC = 0.2; // seconds

  // -------- Odometry and vision fusion objects -------------
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  // (Provide your own initial gyro and module positions)
  private final Rotation2d initialGyro = new Rotation2d();
  private final SwerveModulePosition[] initialModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final SwerveDrivePoseEstimator odometryEstimator;
  private final QuestNav questNav;

  // -------- Buffers for recent pose history (for computing offset derivatives) -------------
  private final TimeInterpolatableBuffer<Pose2d> odometryBuffer;
  private final TimeInterpolatableBuffer<Pose2d> questBuffer;

  // -------- Internal state flags -------------
  // True if we have “synced” (i.e. applied a transformation so that Quest's output is in the field
  // frame).
  private boolean questSynced = false;
  // Time tracking: last time the robot was moving (used for safe re-sync)
  private double lastTimeNotStationary = Timer.getFPGATimestamp();

  public HeimdallPoseController() {
    odometryEstimator =
        new SwerveDrivePoseEstimator(kinematics, initialGyro, initialModulePositions, new Pose2d());

    switch (Constants.currentMode) {
      case REAL:
        questNav = new QuestNav(new QuestNavIOReal());
        break;
      case SIM:
        questNav = new QuestNav(new QuestNavIOSim(odometryEstimator::getEstimatedPosition));
        break;
      default:
        questNav = new QuestNav(new QuestNavIO() {});
    }

    odometryBuffer = TimeInterpolatableBuffer.createBuffer(BUFFER_DURATION);
    questBuffer = TimeInterpolatableBuffer.createBuffer(BUFFER_DURATION);
  }

  /** Update method that should be called each loop with fresh sensor data. */
  public void updateWithTime(
      double currentTimeSeconds,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      ChassisSpeeds chassisSpeeds) {

    // Update odometry and record the pose in our buffer.
    odometryEstimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
    Pose2d odometryPose = odometryEstimator.getEstimatedPosition();
    odometryBuffer.addSample(currentTimeSeconds, odometryPose);

    // If Quest is connected, record its pose.
    if (questNav.isConnected()) {
      Pose2d questPose = questNav.getRobotPose();
      questBuffer.addSample(currentTimeSeconds, questPose);
    }

    // Track whether the robot is moving.
    double normChassisSpeed = computeNormalizedChassisSpeed(chassisSpeeds);
    if (normChassisSpeed > STATIONARY_VELOCITY_THRESHOLD) {
      lastTimeNotStationary = currentTimeSeconds;
    }
    double timeStationary = currentTimeSeconds - lastTimeNotStationary;

    // Evaluate whether to sync or unsync Quest.
    evaluateSyncState(currentTimeSeconds, odometryPose, timeStationary);

    // Log diagnostics.
    Logger.recordOutput("Heimdall/OdometryPose", odometryPose);
    Logger.recordOutput("Heimdall/QuestSynced", questSynced);
    Logger.recordOutput("Heimdall/ChassisSpeedNorm", normChassisSpeed);
    Logger.recordOutput("Heimdall/TimeStationary", timeStationary);
  }

  /**
   * Evaluates whether Quest should be synced (or unsynced) based on the offset between Quest and
   * odometry and whether that offset is steady.
   *
   * @param currentTime The current time.
   * @param odometryPose The current odometry pose.
   * @param timeStationary How long the robot has been stationary.
   */
  private void evaluateSyncState(double currentTime, Pose2d odometryPose, double timeStationary) {
    Logger.recordOutput("Heimdall/QuestConnected", questNav.isConnected());

    if (questSynced) {
      // When synced, check if Quest’s output suddenly drifts.
      if (questNav.isConnected()) {
        Pose2d questPose = questNav.getRobotPose();
        double diff = computePoseDifference(questPose, odometryPose);
        if (diff > MAX_POSE_DIFF) {
          questSynced = false;
          Logger.recordOutput(
              "Heimdall/SyncEvent", "Synced Quest drift exceeded threshold; unsyncing.");
        }
      } else {
        questSynced = false;
        Logger.recordOutput("Heimdall/SyncEvent", "Quest lost connection; unsyncing.");
      }
    } else {
      // When not synced, only consider syncing if Quest is connected.
      if (questNav.isConnected()) {
        double offsetDerivative = computeOffsetDerivative();
        Logger.recordOutput("Heimdall/OffsetDerivative", offsetDerivative);
        Logger.recordOutput("Heimdall/TimeStationary", timeStationary);
        // If the robot is stationary and the offset between Quest and odometry is stable,
        // then re-sync Quest—even if the absolute difference is large.
        if (timeStationary >= MIN_TIME_STATIONARY_FOR_SYNC
            && offsetDerivative < OFFSET_DERIVATIVE_THRESHOLD) {
          // Snap Quest into alignment.
          questNav.setRobotPose(odometryPose);
          questSynced = true;
          Logger.recordOutput("Heimdall/SyncEvent", "Re-synced Quest to odometry (stable offset).");
        }
      }
    }
  }

  /**
   * Returns the current best pose estimate. Uses Quest if it is synced, otherwise falls back to
   * odometry.
   */
  public Pose2d getEstimatedPosition() {
    if (questSynced && questNav.isConnected()) {
      Logger.recordOutput("Heimdall/UsingPoseSource", "Quest");
      return questNav.getRobotPose();
    } else {
      Logger.recordOutput("Heimdall/UsingPoseSource", "Odometry");
      return odometryEstimator.getEstimatedPosition();
    }
  }

  /** Injects a vision measurement (e.g. from an AprilTag) into the odometry estimator. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    odometryEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Resets both odometry and Quest (forcing a sync). */
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    odometryEstimator.resetPosition(gyroAngle, modulePositions, poseMeters);
    questNav.setRobotPose(poseMeters);
    questSynced = true;
    Logger.recordOutput("Heimdall/SyncEvent", "Reset both odometry and Quest to new pose.");
  }

  /**
   * Computes a “difference” between two poses as the sum of the translational distance and the
   * absolute rotation difference (normalized to [–π, π]). You might wish to weight these
   * differently.
   */
  private double computePoseDifference(Pose2d p1, Pose2d p2) {
    double translationDiff = p1.getTranslation().getDistance(p2.getTranslation());
    double rotationDiff =
        Math.abs(normalizeAngle(p1.getRotation().getRadians() - p2.getRotation().getRadians()));
    return translationDiff + rotationDiff;
  }

  /** Normalizes an angle (in radians) to the range [–π, π]. */
  private double normalizeAngle(double angle) {
    return Math.atan2(Math.sin(angle), Math.cos(angle));
  }

  /**
   * Computes a normalized speed based on the translational and rotational components of chassis
   * speeds.
   */
  private double computeNormalizedChassisSpeed(ChassisSpeeds chassisSpeeds) {
    double translationalSpeed =
        Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    double normalizedTranslation = translationalSpeed / DriveConstants.maxSpeedMetersPerSec;
    double maxAngularSpeedRadPerSec =
        DriveConstants.maxSpeedMetersPerSec
            / Math.hypot(DriveConstants.trackWidth / 2.0, DriveConstants.wheelBase / 2.0);
    double normalizedAngular =
        Math.abs(chassisSpeeds.omegaRadiansPerSecond) / maxAngularSpeedRadPerSec;
    return normalizedTranslation + normalizedAngular;
  }

  /**
   * Computes the rate of change (derivative) of the offset between Quest and odometry poses over
   * the buffer window. If the offset is stable, its derivative will be low.
   */
  private double computeOffsetDerivative() {
    var odomMap = odometryBuffer.getInternalBuffer();
    var questMap = questBuffer.getInternalBuffer();
    if (odomMap.size() < 2 || questMap.size() < 2) {
      return Double.POSITIVE_INFINITY; // not enough data
    }

    // Find an overlapping time window between the two buffers.
    double odomStartTime = odomMap.firstEntry().getKey();
    double odomEndTime = odomMap.lastEntry().getKey();
    double questStartTime = questMap.firstEntry().getKey();
    double questEndTime = questMap.lastEntry().getKey();
    double startTime = Math.max(odomStartTime, questStartTime);
    double endTime = Math.min(odomEndTime, questEndTime);
    if (endTime <= startTime) {
      return Double.POSITIVE_INFINITY;
    }

    // Interpolate poses at the start and end of this window.
    Optional<Pose2d> odomStartPose = odometryBuffer.getSample(startTime);
    Optional<Pose2d> odomEndPose = odometryBuffer.getSample(endTime);
    Optional<Pose2d> questStartPose = questBuffer.getSample(startTime);
    Optional<Pose2d> questEndPose = questBuffer.getSample(endTime);
    if (!odomStartPose.isPresent()
        || !odomEndPose.isPresent()
        || !questStartPose.isPresent()
        || !questEndPose.isPresent()) {
      return Double.POSITIVE_INFINITY; // not enough data
    }
    double startOffset = computePoseDifference(questStartPose.get(), odomStartPose.get());
    double endOffset = computePoseDifference(questEndPose.get(), odomEndPose.get());
    double dt = endTime - startTime;
    return Math.abs(endOffset - startOffset) / dt;
  }
}
