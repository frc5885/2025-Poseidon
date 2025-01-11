package frc.robot.subsystems.drive.QuestNav;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class QuestNavConstants {
  // Physical offset from the robot center to the Quest headset.
  // Quest is 14 inches in front of the robot center
  public static final Transform2d robotToQuestTransform =
      new Transform2d(Units.inchesToMeters(14.0), 0, new Rotation2d());
}
