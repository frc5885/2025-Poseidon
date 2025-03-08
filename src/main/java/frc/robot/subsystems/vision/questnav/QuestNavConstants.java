package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class QuestNavConstants {
  // Physical offset from the robot center to the Quest headset.
  public static final Transform2d kRobotToQuestTransform =
      new Transform2d(
          Units.inchesToMeters(10.5), Units.inchesToMeters(5.5), Rotation2d.fromDegrees(60));
}
