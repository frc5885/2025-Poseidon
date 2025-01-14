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

  // Transform to map between the quest's local coordinate system and field coordinates.
  private Rotation2d questToFieldAngleOffset = new Rotation2d();
  private Translation2d questToFieldTranslationOffset = new Translation2d();

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

    Logger.recordOutput("Odometry/Quest", getRobotPose());
    Pose2d questWithCorrectedAngle = new Pose2d(getRawQuestPose().getTranslation(), getRawQuestRotation().plus(questToFieldAngleOffset));
    Logger.recordOutput("Odometry/QuestRaw", getRawQuestPose());
    Logger.recordOutput("Odometry/QuestWithCorrectedAngle", questWithCorrectedAngle);

    Pose2d questWithCorrectedTranslation = new Pose2d(questWithCorrectedAngle.getTranslation().rotateBy(questToFieldAngleOffset).plus(questToFieldTranslationOffset), questWithCorrectedAngle.getRotation());
    Logger.recordOutput("Odometry/QuestWithCorectedTranslation", questWithCorrectedTranslation);

    // Test
    Pose2d robotPoseFromQuest = questWithCorrectedTranslation.transformBy(QuestNavConstants.robotToQuestTransform.inverse());
    Logger.recordOutput("Odometry/RobotPoseFromQuest", robotPoseFromQuest);
  }

  /**
   * Sets the current robot pose in FIELD coordinates. Internally, this calculates and stores the
   * transform needed to map the Quest’s coordinate system to field coordinates.
   *
   * @param realRobotPose The desired Pose2d of the robot in field coordinates
   */
  public void setRobotPose(Pose2d realRobotPose) {
    Pose2d realQuestPose = realRobotPose.transformBy(QuestNavConstants.robotToQuestTransform);
    questToFieldAngleOffset = realQuestPose.getRotation().minus(getRawQuestRotation());
    questToFieldTranslationOffset = realQuestPose.getTranslation().minus(getRawQuestPose().getTranslation());
  }

  /**
   * Returns the current robot pose in FIELD coordinates, incorporating the Quest's local readings,
   * the physical offset of the Quest on the robot, and the transform set by setRobotPose().
   *
   * @return The Pose2d of the robot in field coordinates
   */
  public Pose2d getRobotPose() {
    Pose2d robotPoseInQuestFOR = robotPoseInQuestFOR(getRawQuestPose());
    // return robotPoseInQuestFOR.plus(questToFieldTransform);
    return new Pose2d();
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

  /** Returns the robot pose in the Quest's FOR given the quest's reported pose in its own FOR */
  private Pose2d robotPoseInQuestFOR(Pose2d questPoseInQuestFOR) {
    return questPoseInQuestFOR.transformBy(QuestNavConstants.robotToQuestTransform.inverse());
  }
}
