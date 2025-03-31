package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.FieldConstants.Side;
import java.util.List;
import java.util.Map;

public class PoseUtil {
  public static Map<Pose2d, List<Pose2d>> branchPoses;

  public static int getClosestDesiredBranchID(Pose2d robotPose, Side side) {
    Pose2d closestFace = robotPose.nearest(FieldConstants.getReefFaces());
    int faceID = FieldConstants.getReefFaces().indexOf(closestFace);
    return FieldConstants.getBranchID(faceID, side);
  }

  public static int getBranchIDFromPose(Pose2d branchPose) {
    List<Pose2d> allL4Poses =
        FieldConstants.getAllL4Poses().stream().map(pose -> pose.toPose2d()).toList();
    Pose2d closestL4Pose = branchPose.nearest(allL4Poses);
    return allL4Poses.indexOf(closestL4Pose);
  }
}
