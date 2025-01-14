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

    // quest in quest coords
    Pose2d questInQuestCoords = getRawQuestPose();
    Logger.recordOutput("Odometry/QuestRaw", questInQuestCoords);

    // robot in quest coords
    Pose2d robotInQuestCoords = questInQuestCoords.transformBy(QuestNavConstants.robotToQuestTransform.inverse());
    Logger.recordOutput("Odometry/RobotInQuestCoords", robotInQuestCoords);

    // robot in quest coords rotated
    Pose2d robotInQuestCoordsRotated = new Pose2d(robotInQuestCoords.getTranslation(),
        robotInQuestCoords.getRotation().plus(questToFieldAngleOffset));
    Logger.recordOutput("Odometry/RobotInQuestCoordsRotated", robotInQuestCoordsRotated);

    // robot in field coords (axes corrected)
    Pose2d robotInFieldCoords = new Pose2d(robotInQuestCoords.getTranslation().rotateBy(questToFieldAngleOffset),
        robotInQuestCoordsRotated.getRotation());
    Logger.recordOutput("Odometry/RobotInFieldCoords", robotInFieldCoords);

    // robot in field coords (axes corrected and translated)
    Pose2d robotInFieldCoordsTranslated = new Pose2d(robotInFieldCoords.getTranslation().plus(questToFieldTranslationOffset),
        robotInFieldCoords.getRotation());
    Logger.recordOutput("Odometry/RobotInFieldCoordsFinal", robotInFieldCoordsTranslated);
  }

  /**
   * Sets the current robot pose in FIELD coordinates. Internally, this calculates and stores the
   * transform needed to map the Quest’s coordinate system to field coordinates.
   *
   * @param realRobotPose The desired Pose2d of the robot in field coordinates
   */
  public void setRobotPose(Pose2d realRobotPose) {
    // store quest to field angle
    Pose2d questInQuestCoords = getRawQuestPose();
    Pose2d robotInQuestCoords = questInQuestCoords.transformBy(QuestNavConstants.robotToQuestTransform.inverse());
    questToFieldAngleOffset = realRobotPose.getRotation().minus(robotInQuestCoords.getRotation());

    // store quest to field translation
    Pose2d robotInQuestCoordsRotated = new Pose2d(robotInQuestCoords.getTranslation(),
        robotInQuestCoords.getRotation().plus(questToFieldAngleOffset));
        Pose2d robotInFieldCoords = new Pose2d(robotInQuestCoords.getTranslation().rotateBy(questToFieldAngleOffset),
        robotInQuestCoordsRotated.getRotation());
    questToFieldTranslationOffset = realRobotPose.getTranslation().minus(robotInFieldCoords.getTranslation());
  }

  /**
   * Returns the current robot pose in FIELD coordinates, incorporating the Quest's local readings,
   * the physical offset of the Quest on the robot, and the transform set by setRobotPose().
   *
   * @return The Pose2d of the robot in field coordinates
   */
  public Pose2d getRobotPose() {
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
