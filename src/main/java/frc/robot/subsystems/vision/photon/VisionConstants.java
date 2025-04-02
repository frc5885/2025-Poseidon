// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision.photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class VisionConstants {
  // AprilTag layout - load directly to avoid circular dependency
  public static final AprilTagFieldLayout kAprilTagLayout;

  static {
    AprilTagFieldLayout layout = null;
    try {
      layout =
          new AprilTagFieldLayout(
              Path.of(
                  Filesystem.getDeployDirectory().getPath(),
                  "apriltags",
                  "welded",
                  "2025-both-reefs.json"));
    } catch (IOException e) {
      throw new RuntimeException("Failed to load AprilTag layout", e);
    }
    kAprilTagLayout = layout;
  }

  // Camera names, must match names configured on coprocessor
  public static final String kCamera0Name = "tsunami";
  public static final String kCamera1Name = "sandstorm";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d kRobotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(8.625),
          -Units.inchesToMeters(7.5),
          Units.inchesToMeters(11.875),
          new Rotation3d(0.0, 0, -Units.degreesToRadians(1.09)));
  public static final Transform3d kRobotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(8.625),
          Units.inchesToMeters(7.5),
          Units.inchesToMeters(11.875),
          new Rotation3d(0.0, 0, Units.degreesToRadians(1.09)));

  // Basic filtering thresholds
  public static final double kMaxAmbiguity = 0.3;
  public static final double kMaxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double kLinearStdDevBaseline = 0.05; // Meters
  public static final double kAngularStdDevBaseline = 0.08; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] kCameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static final double kLinearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static final double kAngularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // Map of which AprilTags are trusted for each post
  // to use, call TrustedAprilTags.get(post).get(isFlipped)
  public static final Map<Integer, Map<Boolean, Integer>> TrustedAprilTags = new HashMap<>();

  static {
    // Initialize the nested maps (postID, blueTag, redTag)
    TrustedAprilTags.put(0, Map.of(false, 18, true, 7));
    TrustedAprilTags.put(1, Map.of(false, 18, true, 7));
    TrustedAprilTags.put(2, Map.of(false, 19, true, 6));
    TrustedAprilTags.put(3, Map.of(false, 19, true, 6));
    TrustedAprilTags.put(4, Map.of(false, 20, true, 11));
    TrustedAprilTags.put(5, Map.of(false, 20, true, 11));
    TrustedAprilTags.put(6, Map.of(false, 21, true, 10));
    TrustedAprilTags.put(7, Map.of(false, 21, true, 10));
    TrustedAprilTags.put(8, Map.of(false, 22, true, 9));
    TrustedAprilTags.put(9, Map.of(false, 22, true, 9));
    TrustedAprilTags.put(10, Map.of(false, 17, true, 8));
    TrustedAprilTags.put(11, Map.of(false, 17, true, 8));
  }

  public static List<Integer> lowTags = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

  public static int getTrustedTag(int post, boolean isFlipped) {
    return TrustedAprilTags.get(post).get(isFlipped);
  }

  public static int getTrustedCamera(int post) {
    // even numbered posts are on the right, use left camera (camera 1)
    return 1 - (post % 2);
  }
}
