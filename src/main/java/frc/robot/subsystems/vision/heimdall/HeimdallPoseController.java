// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Special credit to OpenAI's o3-mini-hard

package frc.robot.subsystems.vision.heimdall;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.questnav.QuestNav;
import frc.robot.subsystems.vision.questnav.QuestNavIO;
import frc.robot.subsystems.vision.questnav.QuestNavIOReal;
import org.littletonrobotics.junction.Logger;

public class HeimdallPoseController {
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

  private HeimdallOdometrySource m_mode;

  // Convergence factors - control how quickly the transform is updated
  // Lower values mean slower but smoother convergence
  private static final double kTranslationConvergenceFactor = 0.1; // 10% adjustment per cycle
  private static final double kRotationConvergenceFactor = 0.05; // 5% adjustment per cycle
  private static final double kOptimalDistance =
      0.5; // the distance at which the convergence factors will be highest

  // First-time initialization flag
  private boolean m_isFirstUpdate = true;

  public HeimdallPoseController(HeimdallOdometrySource mode) {
    m_odometryEstimator =
        new SwerveDrivePoseEstimator(
            m_kinematics, m_initialGyro, m_initialModulePositions, new Pose2d());

    switch (Constants.kCurrentMode) {
      case REAL:
        m_questNav = new QuestNav(new QuestNavIOReal());
        break;
      case SIM:
        m_questNav = new QuestNav(new QuestNavIO() {});
        break;
      default:
        m_questNav = new QuestNav(new QuestNavIO() {});
    }

    m_mode = mode;
  }

  /** Update method that should be called each loop with fresh sensor data. */
  public void updateWithTime(
      double currentTimeSeconds,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      ChassisSpeeds chassisSpeeds) {

    // Update odometry
    m_odometryEstimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);

    Logger.recordOutput(
        "Heimdall/SwerveDrivePoseEstimator", m_odometryEstimator.getEstimatedPosition());

    // Note: Quest-to-field transform updates now happen in addVisionMeasurement
  }

  /**
   * Blends two transforms together, moving from the current transform toward the target by the
   * specified convergence factors.
   *
   * @param current The current transform
   * @param target The target transform to converge toward
   * @param translationFactor How much to adjust translation each cycle (0-1)
   * @param rotationFactor How much to adjust rotation each cycle (0-1)
   * @return A new transform that's a blend between current and target
   */
  private Transform2d blendTransforms(
      Transform2d current, Transform2d target, double translationFactor, double rotationFactor) {

    // Calculate the translation difference and apply convergence factor
    Translation2d currentTranslation = current.getTranslation();
    Translation2d targetTranslation = target.getTranslation();
    Translation2d translationDiff = targetTranslation.minus(currentTranslation);

    Translation2d newTranslation =
        currentTranslation.plus(
            new Translation2d(
                translationDiff.getX() * translationFactor,
                translationDiff.getY() * translationFactor));

    // Calculate the rotation difference and apply convergence factor
    // We need to handle angle wrapping properly
    Rotation2d currentRotation = current.getRotation();
    Rotation2d targetRotation = target.getRotation();

    // Find the shortest path rotation difference
    double rotationDiffRad = targetRotation.minus(currentRotation).getRadians();

    // Apply the rotation factor to get the new rotation
    Rotation2d newRotation = currentRotation.plus(new Rotation2d(rotationDiffRad * rotationFactor));

    return new Transform2d(newTranslation, newRotation);
  }

  /**
   * Returns the current best pose estimate. Uses Quest if it is connected, otherwise falls back to
   * odometry.
   */
  public Pose2d getEstimatedPosition() {
    switch (m_mode) {
      case ONLY_APRILTAG_ODOMETRY:
        // Always use odometry
        Logger.recordOutput("Heimdall/UsingPoseSource", "Odometry");
        return m_odometryEstimator.getEstimatedPosition();
      case ONLY_QUEST:
        // Still fall back to odometry if Quest is disconnected
        if (m_questNav.isConnected()) {
          Logger.recordOutput("Heimdall/UsingPoseSource", "Quest");
          return m_questNav.getRobotPose();
        } else {
          Logger.recordOutput("Heimdall/UsingPoseSource", "Odometry");
          return m_odometryEstimator.getEstimatedPosition();
        }
      default:
        // AUTO_SWITCH mode
        if (m_questNav.isConnected()) {
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
      Matrix<N3, N1> visionMeasurementStdDevs,
      double averageTagDistance) {

    // Add vision measurement to odometry estimator
    m_odometryEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);

    // Now update Quest-to-field transform based on the new vision measurement

    // Get the current Quest-to-Field transform
    Transform2d currentQuestToField = m_questNav.getQuestToFieldTransform();

    // Calculate the "target" transform based on the vision pose
    Transform2d targetQuestToField =
        m_questNav.calculateQuestToField(m_odometryEstimator.getEstimatedPosition());

    // If this is the first update, directly set the transform without gradual convergence
    if (m_isFirstUpdate) {
      m_questNav.updateQuestToFieldTransform(targetQuestToField);
      m_isFirstUpdate = false;
      Logger.recordOutput("Heimdall/QuestConvergence", "Initial transform set from vision");
      return;
    }

    // Calculate the new transform by blending the current and target transforms
    Logger.recordOutput("Heimdall/AverageTagDistance", averageTagDistance);
    // Calculate distance-based factor that peaks at optimal distance and falls off symmetrically
    double distanceRatio = Math.abs(averageTagDistance - kOptimalDistance) / kOptimalDistance;
    double factor = Math.max(0.0, Math.min(1.0, 1.0 - distanceRatio));
    double transFactor = kTranslationConvergenceFactor * factor; // higher factor for closer tags
    double rotFactor = kRotationConvergenceFactor * factor; // higher factor for closer tags
    Transform2d newQuestToField =
        blendTransforms(currentQuestToField, targetQuestToField, transFactor, rotFactor);

    // Update QuestNav with the new blended transform
    m_questNav.updateQuestToFieldTransform(newQuestToField);

    Logger.recordOutput("Heimdall/QuestConvergence/CurrentTransform", currentQuestToField);
    Logger.recordOutput("Heimdall/QuestConvergence/BlendedTransform", newQuestToField);
    Logger.recordOutput("Heimdall/QuestConvergence/TargetTransform", targetQuestToField);
  }

  /** Resets both odometry and Quest (forcing a sync). */
  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {
    m_odometryEstimator.resetPosition(gyroAngle, modulePositions, poseMeters);
    m_questNav.setRobotPose(poseMeters);
    m_isFirstUpdate = true; // Reset the first update flag to immediately apply the new transform
    Logger.recordOutput("Heimdall/SyncEvent", "Reset both odometry and Quest to new pose.");
  }

  /** Allows the driver to force sync the quest */
  public void forceSyncQuest() {
    m_questNav.setRobotPose(m_odometryEstimator.getEstimatedPosition());
    m_isFirstUpdate = true; // Reset the first update flag to immediately apply the new transform
    Logger.recordOutput("Heimdall/SyncEvent", "Forced sync of Quest to odometry.");
  }

  /** Update the mode / odometry source */
  public void setMode(HeimdallOdometrySource mode) {
    m_mode = mode;
  }

  public static enum HeimdallOdometrySource {
    ONLY_APRILTAG_ODOMETRY,
    ONLY_QUEST,
    AUTO_SWITCH
  }

  public QuestNav getQuestNav() {
    return m_questNav;
  }
}
