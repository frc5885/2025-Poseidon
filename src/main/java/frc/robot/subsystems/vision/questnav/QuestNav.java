// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Subsystem to interface with a Meta Quest headset acting as a positional sensor. */
public class QuestNav extends SubsystemBase {

  private final QuestNavIO m_questNavIO;
  private final QuestNavIOInputsAutoLogged m_questNavIOInputs = new QuestNavIOInputsAutoLogged();
  private final Alert m_disconnectedAlert;

  // Transform to map between the quest's local coordinate system and field coordinates
  // The translation and rotation get handled separately
  private Transform2d m_questToField = new Transform2d();

  /**
   * Creates a new QuestNav subsystem and uses the robot pose to initialize the quest-to-field
   * transform
   */
  public QuestNav(QuestNavIO questNavIO) {
    m_questNavIO = questNavIO;
    m_disconnectedAlert = new Alert("QuestNav is disconnected.", AlertType.kWarning);
  }

  @Override
  public void periodic() {
    m_questNavIO.updateInputs(m_questNavIOInputs);
    Logger.processInputs("QuestNav", m_questNavIOInputs);
    m_disconnectedAlert.set(!m_questNavIOInputs.connected);
    m_questNavIO.cleanUpQuestNavMessages();
  }

  /**
   * Updates the transform that maps Quest coordinates to field coordinates.
   *
   * <p>This method takes in a robot pose in field coordinates and compares it to the robot’s
   * current pose in Quest coordinates. It then calculates the translation and rotation offsets
   * needed to align Quest coordinates with the field reference frame. The resulting transform is
   * stored for future pose conversions.
   *
   * @param realRobotPose The desired robot pose in field coordinates
   */
  public void setRobotPose(Pose2d realRobotPose) {
    m_questToField = calculateQuestToField(realRobotPose);
  }

  /**
   * Returns the current robot pose in field coordinates.
   *
   * <p>It uses the raw Quest pose, adjusts for the physical offset of the Quest device, and applies
   * the Quest-to-field transform set by {@link #setRobotPose(Pose2d)}. This results in a Pose2d
   * that represents the robot's position and orientation relative to the field's coordinate system.
   *
   * @return The Pose2d of the robot in field coordinates
   */
  public Pose2d getRobotPose() {
    // 1) Pose of the quest in its own coordinates
    Pose2d questInQuestCoords = getRawQuestPose();

    // 2) Transform to get "robot in quest coords" (raw Quest pose transformed by the inverse
    // robot->Quest transform)
    Pose2d robotInQuestCoords =
        questInQuestCoords.transformBy(QuestNavConstants.kRobotToQuestTransform.inverse());

    // 3) Combine rotations: robot's rotation + questToField rotation
    Rotation2d fieldRotation = robotInQuestCoords.getRotation().plus(m_questToField.getRotation());

    // 4) Rotate the translation by questToField rotation
    Translation2d rotatedTranslation =
        robotInQuestCoords.getTranslation().rotateBy(m_questToField.getRotation());

    // 5) Build pose with the updated (rotated) translation and combined rotation
    Pose2d robotInFieldRotated = new Pose2d(rotatedTranslation, fieldRotation);

    // 6) Now apply the questToField translation
    return new Pose2d(
        robotInFieldRotated.getTranslation().plus(m_questToField.getTranslation()),
        robotInFieldRotated.getRotation());
  }

  /**
   * Updates the Quest-to-field transform.
   *
   * @param newTransform
   */
  public void updateQuestToFieldTransform(Transform2d newTransform) {
    m_questToField = newTransform;
  }

  /** Returns the Quest-to-field transform. */
  public Transform2d getQuestToFieldTransform() {
    return m_questToField;
  }

  /** Returns the raw Quest HMD pose (X, Y, Z, rotation) */
  private Pose2d getRawQuestPose() {
    float[] questnavPosition = m_questNavIOInputs.position;
    return new Pose2d(questnavPosition[2], -questnavPosition[0], getRawQuestRotation());
  }

  /** Returns the yaw angle from the Quest's eulerAngles */
  private Rotation2d getRawQuestRotation() {
    float[] eulerAngles = m_questNavIOInputs.eulerAngles;
    float ret = -eulerAngles[1];
    return new Rotation2d(Units.degreesToRadians(ret));
  }

  /** Get whether or not the Quest is connected */
  public boolean isConnected() {
    return m_questNavIOInputs.connected;
  }

  /**
   * Calculates the transform that maps Quest coordinates to field coordinates.
   *
   * <p>This method takes in a robot pose in field coordinates and compares it to the robot’s
   * current pose in Quest coordinates. It then calculates the translation and rotation offsets
   * needed to align Quest coordinates with the field reference frame.
   *
   * @param robotPose The desired robot pose in field coordinates
   * @return The transform that maps Quest coordinates to field coordinates
   */
  public Transform2d calculateQuestToField(Pose2d robotPose) {
    // Get our "robot in quest coordinates" (raw Quest pose transformed by the inverse robot->Quest
    // transform)
    Pose2d robotInQuestCoords =
        getRawQuestPose().transformBy(QuestNavConstants.kRobotToQuestTransform.inverse());

    Rotation2d questToFieldAngleOffset =
    robotPose.getRotation().minus(robotInQuestCoords.getRotation());

    // Rotate the Quest translation by the needed offset, then find how far apart
    // the rotated Quest translation is from the real robot pose in field coords
    Translation2d rotatedQuestTranslation =
        robotInQuestCoords.getTranslation().rotateBy(questToFieldAngleOffset);
    Translation2d questToFieldTranslationOffset =
    robotPose.getTranslation().minus(rotatedQuestTranslation);

    // Create the overall transform
    return new Transform2d(questToFieldTranslationOffset, questToFieldAngleOffset);
  }
}
