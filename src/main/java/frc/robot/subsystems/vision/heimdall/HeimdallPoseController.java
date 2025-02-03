// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.heimdall;

import static frc.robot.subsystems.drive.DriveConstants.moduleTranslations;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
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
import java.util.NavigableMap;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class HeimdallPoseController {
  // generic default values for pose estimator
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private final Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private final SwerveDrivePoseEstimator odometryPoseEstimator;
  private final QuestNav questNav;

  private final double maxSpeedMetersPerSec = DriveConstants.maxSpeedMetersPerSec;
  private final double maxAngularSpeedRadPerSec =
      maxSpeedMetersPerSec
          / (Math.hypot(DriveConstants.trackWidth / 2.0, DriveConstants.wheelBase / 2.0));

  // flags
  private boolean questSynced = false;

  // thresholds
  // how much faster odometry can move than quest without triggering a re-sync (in m/s + rad/s)
  private double maxAllowableQuestVelocityDiff = 0.4;
  // how much difference in pose between quest and odometry is allowed before re-syncing (in m +
  // rad)
  private double maxAllowablePoseDiff = 0.4;
  // how much motion/jitter is allowed from odometry in order to safely re-sync quest (in m/s +
  // rad/s)
  private double maxAllowableOdometryVelocityDiff = 0.02;
  // how much motion is allowed from the chassis speeds for the robot to be considered stationary
  // (in m/s + rad/s)
  private double maxAllowableChassisSpeedsVelocity = 0.001;

  // history of observations
  private final double bufferLength = 0.4;
  private final TimeInterpolatableBuffer<Pose2d> odometryPosesHistory;
  private final TimeInterpolatableBuffer<Pose2d> questPosesHistory;
  private final TimeInterpolatableBuffer<Double> chassisVelocityHistory;
  private double lastVisionMeasurementTime = 0.0;

  private double lastQuestVelocity = 0.0;
  private double lastOdometryVelocity = 0.0;
  private double lastChassisSpeedsVelocity = 0.0;

  private double lastAttemptSyncTime = 0.0;

  public HeimdallPoseController() {
    odometryPoseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    switch (Constants.currentMode) {
      case REAL:
        questNav = new QuestNav(new QuestNavIOReal() {});
        break;
      case SIM:
        questNav = new QuestNav(new QuestNavIOSim(odometryPoseEstimator::getEstimatedPosition) {});
        break;
      default:
        questNav = new QuestNav(new QuestNavIO() {});
    }
    odometryPosesHistory = TimeInterpolatableBuffer.createBuffer(bufferLength);
    questPosesHistory = TimeInterpolatableBuffer.createBuffer(bufferLength);
    chassisVelocityHistory = TimeInterpolatableBuffer.createDoubleBuffer(bufferLength);
  }

  public void periodic() {
    if (questNav.isConnected()) {
      questPosesHistory.addSample(Timer.getTimestamp(), questNav.getRobotPose());
    }
    odometryPosesHistory.addSample(
        Timer.getTimestamp(), odometryPoseEstimator.getEstimatedPosition());

    lastQuestVelocity = getVelocityMagnitudeFromBuffer(questPosesHistory);
    Logger.recordOutput("Odometry/QuestNavVelocity", lastQuestVelocity);

    lastOdometryVelocity = getVelocityMagnitudeFromBuffer(odometryPosesHistory);
    Logger.recordOutput("Odometry/OdometryVelocity", lastOdometryVelocity);

    double questVelocityDiff = Math.max(lastChassisSpeedsVelocity - lastQuestVelocity, 0);
    Logger.recordOutput("Odometry/QuestVelocityDiff", questVelocityDiff);

    double odometryVelocityDiff = Math.abs(lastChassisSpeedsVelocity - lastOdometryVelocity);
    Logger.recordOutput("Odometry/OdometryVelocityDiff", odometryVelocityDiff);

    boolean recentVisionMeasurement =
        Timer.getTimestamp() - lastVisionMeasurementTime < bufferLength;
    Logger.recordOutput("Odometry/RecentVisionMeasurement", recentVisionMeasurement);

    if (questSynced && questVelocityDiff > maxAllowableQuestVelocityDiff) {
      // quest has fallen behind, re-sync
      questSynced = false;
    }

    double poseDiff = 0.0;
    if (questPosesHistory.getInternalBuffer().size() > 2
        && odometryPosesHistory.getInternalBuffer().size() > 2) {
      Pose2d lastQuestPose = questPosesHistory.getInternalBuffer().lastEntry().getValue();
      Pose2d lastOdometryPose = odometryPosesHistory.getInternalBuffer().lastEntry().getValue();
      poseDiff =
          lastQuestPose.getTranslation().getDistance(lastOdometryPose.getTranslation())
              + normalizeAngleDelta(
                  lastQuestPose.getRotation().getRadians()
                      - lastOdometryPose.getRotation().getRadians());
    }
    Logger.recordOutput("Odometry/PoseDiff", poseDiff);

    if (questSynced && poseDiff > maxAllowablePoseDiff && recentVisionMeasurement) {
      // pose has diverged, re-sync
      questSynced = false;
    }

    if (!questSynced) {
      attemptQuestSyncIfSafe(
          lastChassisSpeedsVelocity, odometryVelocityDiff, recentVisionMeasurement);
    }
  }

  public void syncQuest() {
    questNav.setRobotPose(odometryPoseEstimator.getEstimatedPosition());
    System.out.println("Synced Quest to Odometry");
    questSynced = true;
  }

  private void attemptQuestSyncIfSafe(
      double chassisSpeedsVelocity, double odometryVelocityDiff, boolean recentVisionMeasurement) {
    double currentTime = Timer.getTimestamp();
    boolean conditionsMet =
        chassisSpeedsVelocity < maxAllowableChassisSpeedsVelocity
            && recentVisionMeasurement
            && odometryVelocityDiff < maxAllowableOdometryVelocityDiff
            && odometryVelocityDiff != 0.0; // if it's truly zero it's not processing yet

    if (conditionsMet) {
      if (currentTime - lastAttemptSyncTime >= bufferLength) {
        syncQuest();
      }
    } else {
      lastAttemptSyncTime = currentTime;
    }
  }

  private double chassisSpeedsToVelocityMagnitude(ChassisSpeeds chassisSpeeds) {
    return Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            / DriveConstants.maxSpeedMetersPerSec
        + Math.abs(chassisSpeeds.omegaRadiansPerSecond) / maxAngularSpeedRadPerSec;
  }

  private double getVelocityMagnitudeFromBuffer(TimeInterpolatableBuffer<Pose2d> poseHistory) {
    NavigableMap<Double, Pose2d> buffer = poseHistory.getInternalBuffer();
    if (buffer.size() < 2) {
      return 0.0;
    }
    double t1 = buffer.firstEntry().getKey();
    double t2 = buffer.lastEntry().getKey();
    Pose2d pose1 = buffer.firstEntry().getValue();
    Pose2d pose2 = buffer.lastEntry().getValue();

    double angleDelta = pose2.getRotation().getRadians() - pose1.getRotation().getRadians();
    angleDelta = normalizeAngleDelta(angleDelta);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            (pose2.getTranslation().getX() - pose1.getTranslation().getX()) / (t2 - t1),
            (pose2.getTranslation().getY() - pose1.getTranslation().getY()) / (t2 - t1),
            angleDelta / (t2 - t1));
    return chassisSpeedsToVelocityMagnitude(speeds);
  }

  private double normalizeAngleDelta(double angleDelta) {
    return Math.atan2(Math.sin(angleDelta), Math.cos(angleDelta)); // Normalize to [-π, π]
  }

  // ---------- Wrappers for SwerveDrivePoseEstimator methods ---------- //
  /**
   * Gets the estimated robot pose.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getEstimatedPosition() {
    // this gets called periodically
    periodic();
    boolean useQuest = questNav.isConnected() && questSynced;
    Logger.recordOutput("Odometry/UsingQuest", useQuest);
    return useQuest ? questNav.getRobotPose() : odometryPoseEstimator.getEstimatedPosition();
  }

  /**
   * Updates the pose estimator with wheel encoder and gyro information. This should be called every
   * loop.
   *
   * @param gyroAngle The current gyro angle.
   * @param wheelPositions The current encoder readings.
   * @return The estimated pose of the robot in meters.
   */
  public void updateWithTime(
      double currentTimeSeconds,
      Rotation2d gyroAngle,
      SwerveModulePosition[] wheelPositions,
      ChassisSpeeds chassisSpeeds) {
    Pose2d updatedPose =
        odometryPoseEstimator.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);

    lastChassisSpeedsVelocity = chassisSpeedsToVelocityMagnitude(chassisSpeeds);
    chassisVelocityHistory.addSample(currentTimeSeconds, lastChassisSpeedsVelocity);
    Logger.recordOutput("Odometry/VisionAndSensors", updatedPose);
    Logger.recordOutput("Odometry/ChassisVelocity", lastChassisSpeedsVelocity);
  }
  /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @param poseMeters The position on the field that your robot is at.
   */
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
    odometryPoseEstimator.resetPosition(gyroAngle, wheelPositions, poseMeters);
    syncQuest();
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   *
   * <p>This method can be called as infrequently as you want, as long as you are calling {@link
   * PoseEstimator#update} every loop.
   *
   * <p>To promote stability of the pose estimate and make it robust to bad vision data, we
   * recommend only adding vision measurements that are already within one meter or so of the
   * current pose estimate.
   *
   * <p>Note that the vision measurement standard deviations passed into this method will continue
   * to apply to future measurements until a subsequent call to {@link
   * PoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
   *
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds. Note that if you
   *     don't use your own time source by calling {@link #updateWithTime}, then you must use a
   *     timestamp with an epoch since FPGA startup (i.e., the epoch of this timestamp is the same
   *     epoch as {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}). This means that you
   *     should use {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()} as your time source in
   *     this case.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement (x position
   *     in meters, y position in meters, and heading in radians). Increase these numbers to trust
   *     the vision pose measurement less.
   */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    odometryPoseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);

    // Update the last vision measurement time
    lastVisionMeasurementTime = timestampSeconds;
  }
}
