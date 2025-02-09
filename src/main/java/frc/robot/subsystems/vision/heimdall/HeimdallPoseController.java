// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Special credit to OpenAI's o3-mini-hard

package frc.robot.subsystems.vision.heimdall;

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

  // We consider the robot "stationary" if the normalized chassis speed is below this.
  private static final double STATIONARY_VELOCITY_THRESHOLD = 0.05;
  // And we require the robot to be stationary for at least this long before re-syncing.
  private static final double MIN_TIME_STATIONARY_FOR_SYNC = 0.2; // seconds

  // -------- Odometry and vision fusion objects -------------
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(DriveConstants.kModuleTranslations);
  private final Rotation2d m_initialGyro = new Rotation2d();
  private final SwerveModulePosition[] m_initialModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final SwerveDrivePoseEstimator m_odometryEstimator;
  private final QuestNav m_questNav;

  // -------- Buffers for recent pose history (for computing offset derivatives) -------------
  private final TimeInterpolatableBuffer<Pose2d> m_odometryBuffer;
  private final TimeInterpolatableBuffer<Pose2d> m_questBuffer;

  // -------- Internal state flags -------------
  // True if we have "synced" (i.e. applied a transformation so that Quest's output is in the field
  // frame).
  private boolean m_questSynced = false;
  // Time tracking: last time the robot was moving (used for safe re-sync)
  private double m_lastTimeNotStationary = Timer.getFPGATimestamp();

  private HeimdallOdometrySource m_mode;

  public HeimdallPoseController(HeimdallOdometrySource mode) {
    m_odometryEstimator =
        new SwerveDrivePoseEstimator(
            m_kinematics, m_initialGyro, m_initialModulePositions, new Pose2d());

    switch (Constants.kCurrentMode) {
      case REAL:
        m_questNav = new QuestNav(new QuestNavIOReal());
        break;
      case SIM:
        m_questNav = new QuestNav(new QuestNavIOSim(m_odometryEstimator::getEstimatedPosition));
        break;
      default:
        m_questNav = new QuestNav(new QuestNavIO() {});
    }

    m_odometryBuffer = TimeInterpolatableBuffer.createBuffer(BUFFER_DURATION);
    m_questBuffer = TimeInterpolatableBuffer.createBuffer(BUFFER_DURATION);

    m_mode = mode;
  }

  /** Update method that should be called each loop with fresh sensor data. */
  public void updateWithTime(
      double currentTimeSeconds,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      ChassisSpeeds chassisSpeeds) {

    // Update odometry and record the pose in our buffer.
    m_odometryEstimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
    Pose2d odometryPose = m_odometryEstimator.getEstimatedPosition();
    m_odometryBuffer.addSample(currentTimeSeconds, odometryPose);

    // If Quest is connected, record its pose.
    if (m_questNav.isConnected()) {
      Pose2d questPose = m_questNav.getRobotPose();
      m_questBuffer.addSample(currentTimeSeconds, questPose);
      Logger.recordOutput("Heimdall/QuestNav", questPose);
    }

    if (m_mode == HeimdallOdometrySource.AUTO_SWITCH) {
      // Track whether the robot is moving.
      double normChassisSpeed = computeNormalizedChassisSpeed(chassisSpeeds);
      if (normChassisSpeed > STATIONARY_VELOCITY_THRESHOLD) {
        m_lastTimeNotStationary = currentTimeSeconds;
      }
      double timeStationary = currentTimeSeconds - m_lastTimeNotStationary;

      // Evaluate whether to sync or unsync Quest.
      evaluateSyncState(currentTimeSeconds, odometryPose, timeStationary);
    }

    // Log diagnostics.
    Logger.recordOutput("Heimdall/SwerveDrivePoseEstimator", odometryPose);
    Logger.recordOutput("Heimdall/QuestSynced", m_questSynced);
    // Logger.recordOutput("Heimdall/ChassisSpeedNorm", normChassisSpeed);
    // Logger.recordOutput("Heimdall/TimeStationary", timeStationary);
  }

  /** Evaluates whether Quest should be synced or unsynced. */
  private void evaluateSyncState(double currentTime, Pose2d odometryPose, double timeStationary) {

    if (m_questSynced) {
      // If synced, check if the Quest pose has drifted.
      if (m_questNav.isConnected()) {
        Pose2d questPose = m_questNav.getRobotPose();
        double diff = computePoseDifference(questPose, odometryPose);
        if (diff > MAX_POSE_DIFF) {
          m_questSynced = false;
          Logger.recordOutput(
              "Heimdall/SyncEvent", "Synced Quest drift exceeded threshold; unsyncing.");
        }
      } else {
        m_questSynced = false;
        Logger.recordOutput("Heimdall/SyncEvent", "Quest lost connection; unsyncing.");
      }
    } else {
      // Only consider syncing if we've been stationary long enough.
      if (!m_questNav.isConnected() || timeStationary < MIN_TIME_STATIONARY_FOR_SYNC) {
        return;
      }
      // Compute offset derivative only when needed.
      double offsetDerivative = computeOffsetDerivative();
      Logger.recordOutput("Heimdall/OffsetDerivative", offsetDerivative);
      Logger.recordOutput("Heimdall/TimeStationary", timeStationary);
      if (offsetDerivative < OFFSET_DERIVATIVE_THRESHOLD) {
        // Snap Quest into alignment.
        m_questNav.setRobotPose(odometryPose);
        m_questSynced = true;
        Logger.recordOutput("Heimdall/SyncEvent", "Re-synced Quest to odometry (stable offset).");
      }
    }
  }

  /**
   * Returns the current best pose estimate. Uses Quest if it is synced, otherwise falls back to
   * odometry.
   */
  public Pose2d getEstimatedPosition() {
    switch (m_mode) {
      case ONLY_APRILTAG_ODOMETRY:
        Logger.recordOutput("Heimdall/UsingPoseSource", "Odometry");
        return m_odometryEstimator.getEstimatedPosition();
      case ONLY_QUEST:
        Logger.recordOutput("Heimdall/UsingPoseSource", "Quest");
        return m_questNav.getRobotPose();
      default:
        if (m_questSynced && m_questNav.isConnected()) {
          Logger.recordOutput("Heimdall/UsingPoseSource", "Quest");
          return m_questNav.getRobotPose();
        } else {
          Logger.recordOutput("Heimdall/UsingPoseSource", "Odometry");
          return m_odometryEstimator.getEstimatedPosition();
        }
    }
  }

  /** Injects a vision measurement (e.g. from an AprilTag) into the odometry estimator. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    m_odometryEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Resets both odometry and Quest (forcing a sync). */
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    m_odometryEstimator.resetPosition(gyroAngle, modulePositions, poseMeters);
    m_questNav.setRobotPose(poseMeters);
    m_questSynced = true;
    Logger.recordOutput("Heimdall/SyncEvent", "Reset both odometry and Quest to new pose.");
  }

  /** Allows the driver to force sync the quest */
  public void forceSyncQuest() {
    m_questSynced = true;
    m_questNav.setRobotPose(m_odometryEstimator.getEstimatedPosition());
    Logger.recordOutput("Heimdall/SyncEvent", "Forced sync of Quest to odometry.");
  }

  /** Update the mode / odometry source */
  public void setMode(HeimdallOdometrySource mode) {
    m_mode = mode;
  }

  /**
   * Computes a "difference" between two poses as the sum of the translational distance and the
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
    double normalizedTranslation = translationalSpeed / DriveConstants.kMaxSpeedMetersPerSec;
    double maxAngularSpeedRadPerSec =
        DriveConstants.kMaxSpeedMetersPerSec
            / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0);
    double normalizedAngular =
        Math.abs(chassisSpeeds.omegaRadiansPerSecond) / maxAngularSpeedRadPerSec;
    return normalizedTranslation + normalizedAngular;
  }

  /**
   * Computes the rate of change (derivative) of the offset between Quest and odometry poses over
   * the buffer window. If the offset is stable, its derivative will be low.
   */
  private double computeOffsetDerivative() {
    var odomMap = m_odometryBuffer.getInternalBuffer();
    var questMap = m_questBuffer.getInternalBuffer();
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
    Optional<Pose2d> odomStartPose = m_odometryBuffer.getSample(startTime);
    Optional<Pose2d> odomEndPose = m_odometryBuffer.getSample(endTime);
    Optional<Pose2d> questStartPose = m_questBuffer.getSample(startTime);
    Optional<Pose2d> questEndPose = m_questBuffer.getSample(endTime);
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

  public static enum HeimdallOdometrySource {
    ONLY_APRILTAG_ODOMETRY,
    ONLY_QUEST,
    AUTO_SWITCH
  }
}
