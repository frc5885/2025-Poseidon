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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.questnav.QuestNav;

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
  private QuestNav questPoseEstimator;
  private ChassisSpeeds chassisSpeeds;

  public HeimdallPoseController() {
    odometryPoseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    chassisSpeeds = new ChassisSpeeds();
  }

  public void registerQuestNav(QuestNav questNav) {
    this.questPoseEstimator = questNav;
  }

  /**
   * Gets the estimated robot pose.
   *
   * @return The estimated robot pose in meters.
   */
  public Pose2d getEstimatedPosition() {
    // this gets called periodically
    return odometryPoseEstimator.getEstimatedPosition();
  }

  // ---------- Wrappers for SwerveDrivePoseEstimator methods ---------- //
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
    odometryPoseEstimator.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
    this.chassisSpeeds = chassisSpeeds;
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
  }
}
