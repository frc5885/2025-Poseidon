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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Scoring Pose Constants
  public static final double kReefAdditionalDistance = 11.80 - 3.23;

  // Blue Scoring Poses
  public static final Pose2d[] kBluePoses = {
    new Pose2d(3.23, 3.86, new Rotation2d(0)),
    new Pose2d(3.23, 4.19, new Rotation2d(0)),
    new Pose2d(3.70, 5.05, new Rotation2d(5.24)),
    new Pose2d(4.00, 5.25, new Rotation2d(5.24)),
    new Pose2d(4.98, 5.21, new Rotation2d(4.19)),
    new Pose2d(5.27, 5.04, new Rotation2d(4.19)),
    new Pose2d(5.76, 3.86, new Rotation2d(3.14)),
    new Pose2d(5.76, 4.19, new Rotation2d(3.14)),
    new Pose2d(5.28, 3.01, new Rotation2d(2.10)),
    new Pose2d(4.99, 2.84, new Rotation2d(2.10)),
    new Pose2d(3.99, 2.84, new Rotation2d(1.05)),
    new Pose2d(3.71, 3.01, new Rotation2d(1.05))
  };

  // Red Scoring Poses
  public static final Pose2d[] kRedPoses = {
    new Pose2d(3.23 + kReefAdditionalDistance, 3.86, new Rotation2d(0)),
    new Pose2d(3.23 + kReefAdditionalDistance, 4.19, new Rotation2d(0)),
    new Pose2d(3.70 + kReefAdditionalDistance, 5.05, new Rotation2d(5.24)),
    new Pose2d(3.23 + kReefAdditionalDistance, 4.19, new Rotation2d(5.24)),
    new Pose2d(4.98 + kReefAdditionalDistance, 5.21, new Rotation2d(4.19)),
    new Pose2d(5.27 + kReefAdditionalDistance, 5.04, new Rotation2d(4.19)),
    new Pose2d(5.76 + kReefAdditionalDistance, 3.86, new Rotation2d(3.14)),
    new Pose2d(5.76 + kReefAdditionalDistance, 4.19, new Rotation2d(3.14)),
    new Pose2d(5.28 + kReefAdditionalDistance, 3.01, new Rotation2d(8.38)),
    new Pose2d(4.99 + kReefAdditionalDistance, 2.84, new Rotation2d(8.38)),
    new Pose2d(3.99 + kReefAdditionalDistance, 2.84, new Rotation2d(8.38)),
    new Pose2d(3.71 + kReefAdditionalDistance, 3.01, new Rotation2d(8.38))
  };

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
