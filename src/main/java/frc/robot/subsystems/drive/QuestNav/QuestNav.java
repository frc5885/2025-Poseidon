// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.QuestNav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Subsystem to interface with a Meta Quest headset acting as a positional sensor. */
public class QuestNav extends SubsystemBase {

  private final QuestNavIO questNavIO;
  private final QuestNavIOInputsAutoLogged questNavIOInputs = new QuestNavIOInputsAutoLogged();

  // Transform to map between the quest's local coordinate system and field coordinates.
  private Transform2d questToFieldTransform = new Transform2d();

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
    questNavIO.cleanUpQuestNavMessages();

    Logger.recordOutput("Odometry/Quest", getRobotPose());
  }

  /**
   * Sets the current robot pose in FIELD coordinates. Internally, this calculates and stores the
   * transform needed to map the Quest’s coordinate system to field coordinates.
   *
   * @param realRobotPose The desired Pose2d of the robot in field coordinates
   */
  public void setRobotPose(Pose2d realRobotPose) {
    Pose2d realQuestPose = realRobotPose.transformBy(QuestNavConstants.robotToQuestTransform);
    questToFieldTransform = realQuestPose.minus(getRawQuestPose());
  }

  /**
   * Returns the current robot pose in FIELD coordinates, incorporating the Quest's local readings,
   * the physical offset of the Quest on the robot, and the transform set by setRobotPose().
   *
   * @return The Pose2d of the robot in field coordinates
   */
  public Pose2d getRobotPose() {
    return getRawQuestPose()
        .transformBy(questToFieldTransform)
        .transformBy(QuestNavConstants.robotToQuestTransform.inverse());
  }

  /** Returns the raw Quest HMD pose (X, Y, Z, rotation) */
  public Pose2d getRawQuestPose() {
    float[] questnavPosition = questNavIOInputs.position;
    return new Pose2d(questnavPosition[2], -questnavPosition[0], getRawQuestRotation());
  }

  /** Returns the yaw angle from the Quest's eulerAngles */
  public Rotation2d getRawQuestRotation() {
    float[] eulerAngles = questNavIOInputs.eulerAngles;
    float ret = eulerAngles[1];
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return new Rotation2d(Units.degreesToRadians(ret));
  }
}
