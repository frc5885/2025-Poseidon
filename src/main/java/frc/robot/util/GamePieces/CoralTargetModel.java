package frc.robot.util.GamePieces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.photonvision.estimation.TargetModel;

public class CoralTargetModel {
  public static TargetModel getCoralModel() {
    return new TargetModel(getVertices());
  }

  public static List<Translation3d> getVertices() {
    List<Translation3d> vertices = new ArrayList<>();
    double radius = 0.055; // 11cm diameter -> 5.5cm radius
    double halfLength = 0.15; // 30cm length -> ±15cm along X-axis
    int numSegments = 8; // Number of points around the circle

    // Generate points for the two end circles
    for (int i = 0; i < numSegments; i++) {
      double angle = 2.0 * Math.PI * i / numSegments;
      double y = radius * Math.cos(angle);
      double z = radius * Math.sin(angle);

      // One circle at x = -0.15 (left end)
      vertices.add(new Translation3d(-halfLength, y, z));

      // One circle at x = +0.15 (right end)
      vertices.add(new Translation3d(halfLength, y, z));
    }

    return vertices;
  }

  public static List<Pose2d> getCoralPositions() {
    // positions near human player stations, randomized positions and rotations
    List<Pose2d> positions =
        Arrays.asList(
            new Pose2d(
                1.5 + Math.random() * 0.6 - 0.3,
                6.9 + Math.random() * 0.6 - 0.3,
                new Rotation2d(Math.random() * Math.PI)),
            new Pose2d(
                1.5 + Math.random() * 0.6 - 0.3,
                1.3 + Math.random() * 0.6 - 0.3,
                new Rotation2d(Math.random() * Math.PI)),
            new Pose2d(
                16.2 + Math.random() * 0.6 - 0.3,
                7.9 + Math.random() * 0.6 - 0.3,
                new Rotation2d(Math.random() * Math.PI)),
            new Pose2d(
                16.2 + Math.random() * 0.6 - 0.3,
                1.3 + Math.random() * 0.6 - 0.3,
                new Rotation2d(Math.random() * Math.PI)));
    return positions;
  }
}
