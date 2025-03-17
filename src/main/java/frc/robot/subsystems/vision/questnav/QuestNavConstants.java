package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class QuestNavConstants {
  // Physical offset from the robot center to the Quest headset.
  public static final Transform2d kRobotToQuestTransform =
      new Transform2d(0.062, 0.251, Rotation2d.fromDegrees(60.0));
}
