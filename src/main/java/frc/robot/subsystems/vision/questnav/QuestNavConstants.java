package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class QuestNavConstants {
  // Physical offset from the robot center to the Quest headset.
  public static final Transform2d kRobotToQuestTransform =
      new Transform2d(
          Units.inchesToMeters(8.9), Units.inchesToMeters(4.4), Rotation2d.fromDegrees(60));
}
