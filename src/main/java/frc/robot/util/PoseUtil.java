package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Map;

public class PoseUtil {
  public static Map<Pose2d, List<Pose2d>> branchPoses;

  static {
    // branchPoses.put(
    //     FieldConstants.Reef.centerFaces[0], List.of(FieldConstants.Reef.branchPositions.get));
  }

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

  public static Pose2d getTargetPose(Pose2d centerFace) {

    return null;
  }
}
