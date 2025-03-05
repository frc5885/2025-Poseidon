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
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static final AprilTagFieldLayout kAprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final int kCameraPowerId = 40;
  //   public static final int kCameraPowerChannel = 1;

  // Camera names, must match names configured on coprocessor
  public static final String kCamera0Name = "tsunami";
  public static final String kCamera1Name = "sandstorm";

  public static final String kCamera2Name = "coral";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static final Transform3d kRobotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(10),
          -Units.inchesToMeters(12.25),
          Units.inchesToMeters(8.0),
          new Rotation3d(0.0, 0, Units.degreesToRadians(10)));
  public static final Transform3d kRobotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(10),
          Units.inchesToMeters(12.25),
          Units.inchesToMeters(8.0),
          new Rotation3d(0.0, 0, -Units.degreesToRadians(10)));
  // coral intake camera
  public static final Transform3d kRobotToCamera2 =
      new Transform3d(0, 0, 1, new Rotation3d(0, Units.degreesToRadians(50), Math.PI));

  // Basic filtering thresholds
  public static final double kMaxAmbiguity = 0.3;
  public static final double kMaxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double kLinearStdDevBaseline = 0.25; // Meters
  public static final double kAngularStdDevBaseline = 0.6; // Radians

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
}
