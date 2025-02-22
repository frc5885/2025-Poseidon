package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseUtil {

  public static Pose2d findClosestPose(Pose2d[] poses, Pose2d target) {
    Pose2d closestPose = poses[0];
    double closestDistance = target.getTranslation().getDistance(closestPose.getTranslation());
    for (int i = 1; i < poses.length; i++) {
      double distance = target.getTranslation().getDistance(poses[i].getTranslation());
      if (distance < closestDistance) {
        closestPose = poses[i];
        closestDistance = distance;
      }
    }
    return closestPose;
  }
}
