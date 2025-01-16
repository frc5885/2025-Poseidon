// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.QuestNav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Subsystem to interface with a Meta Quest headset acting as a positional sensor. */
public class QuestNav extends SubsystemBase {

  private final QuestNavIO questNavIO;
  private final QuestNavIOInputsAutoLogged questNavIOInputs = new QuestNavIOInputsAutoLogged();

  // Transform to map between the quest's local coordinate system and field coordinates
  // The translation and rotation get handled separately
  private Transform2d questToField = new Transform2d();

  /**
   * Creates a new QuestNav subsystem and uses the robot pose to initalize the quest-to-field
   * transform
   */
  public QuestNav(QuestNavIO questNavIO, Pose2d realRobotPose) {
    this.questNavIO = questNavIO;
    setRobotPose(realRobotPose);
  }

  @Override
  public void periodic() {
    questNavIO.updateInputs(questNavIOInputs);
    Logger.processInputs("QuestNav", questNavIOInputs);
    questNavIO.cleanUpQuestNavMessages();

    Logger.recordOutput("Odometry/QuestNavRobot", getRobotPose());
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
    // Get our "robot in quest coordinates" (raw Quest pose transformed by the inverse robot->Quest
    // transform)
    Pose2d robotInQuestCoords =
        getRawQuestPose().transformBy(QuestNavConstants.robotToQuestTransform.inverse());

    Rotation2d questToFieldAngleOffset =
        realRobotPose.getRotation().minus(robotInQuestCoords.getRotation());

    // Rotate the Quest translation by the needed offset, then find how far apart
    // the rotated Quest translation is from the real robot pose in field coords
    Translation2d rotatedQuestTranslation =
        robotInQuestCoords.getTranslation().rotateBy(questToFieldAngleOffset);
    Translation2d questToFieldTranslationOffset =
        realRobotPose.getTranslation().minus(rotatedQuestTranslation);

    // Create and store the overall transform
    questToField = new Transform2d(questToFieldTranslationOffset, questToFieldAngleOffset);
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
        questInQuestCoords.transformBy(QuestNavConstants.robotToQuestTransform.inverse());

    // 3) Combine rotations: robot's rotation + questToField rotation
    Rotation2d fieldRotation = robotInQuestCoords.getRotation().plus(questToField.getRotation());

    // 4) Rotate the translation by questToField rotation
    Translation2d rotatedTranslation =
        robotInQuestCoords.getTranslation().rotateBy(questToField.getRotation());

    // 5) Build pose with the updated (rotated) translation and combined rotation
    Pose2d robotInFieldRotated = new Pose2d(rotatedTranslation, fieldRotation);

    // 6) Now apply the questToField translation
    return new Pose2d(
        robotInFieldRotated.getTranslation().plus(questToField.getTranslation()),
        robotInFieldRotated.getRotation());
  }

  /** Returns the raw Quest HMD pose (X, Y, Z, rotation) */
  private Pose2d getRawQuestPose() {
    float[] questnavPosition = questNavIOInputs.position;
    return new Pose2d(questnavPosition[2], -questnavPosition[0], getRawQuestRotation());
  }

  /** Returns the yaw angle from the Quest's eulerAngles */
  private Rotation2d getRawQuestRotation() {
    float[] eulerAngles = questNavIOInputs.eulerAngles;
    float ret = -eulerAngles[1];
    return new Rotation2d(Units.degreesToRadians(ret));
  }
}
