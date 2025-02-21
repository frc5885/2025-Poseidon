// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import java.io.IOException;
import java.nio.file.Path;
import java.util.*;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final FieldType fieldType = FieldType.WELDED;

  // Replace with:
  public static final double fieldLength;
  public static final double fieldWidth;

  // Add static initializer block
  static {
    AprilTagFieldLayout layout = AprilTagLayoutType.OFFICIAL.getLayout();
    fieldLength = layout.getFieldLength();
    fieldWidth = layout.getFieldWidth();
  }

  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double algaeDiameter = Units.inchesToMeters(16);

  public static class Processor {
    public static final Pose2d centerFace =
        AprilTagLayoutType.OFFICIAL
            .getLayout()
            .getTagPose(16)
            .get()
            .toPose2d()
            .plus(new Transform2d(0.65, 0.0, new Rotation2d(Math.PI)));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final double stationLength = Units.inchesToMeters(79.750);
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
    public static final Pose2d leftCenterFace =
        new Pose2d(
            rightCenterFace.getX(),
            fieldWidth - rightCenterFace.getY(),
            Rotation2d.fromRadians(-rightCenterFace.getRotation().getRadians()));
  }

  public static class Reef {
    public static final double faceLength = Units.inchesToMeters(36.792600);
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the reef in clockwise order
    public static final SuperStructureState[] AlgaeLevel = new SuperStructureState[6];
    public static final double centerFaceOffset = 0.65;
    public static final List<Map<ReefLevel, Pose3d>> branchPositions =
        new ArrayList<>(); // Starting at the right branch facing the reef in clockwise

    static {
      // Initialize faces
      var aprilTagLayout = AprilTagLayoutType.OFFICIAL.getLayout();
      centerFaces[0] =
          aprilTagLayout
              .getTagPose(18)
              .get()
              .toPose2d()
              .plus(new Transform2d(centerFaceOffset, 0.0, new Rotation2d(Math.PI)));
      centerFaces[1] =
          aprilTagLayout
              .getTagPose(19)
              .get()
              .toPose2d()
              .plus(new Transform2d(centerFaceOffset, 0.0, new Rotation2d(Math.PI)));
      centerFaces[2] =
          aprilTagLayout
              .getTagPose(20)
              .get()
              .toPose2d()
              .plus(new Transform2d(centerFaceOffset, 0.0, new Rotation2d(Math.PI)));
      centerFaces[3] =
          aprilTagLayout
              .getTagPose(21)
              .get()
              .toPose2d()
              .plus(new Transform2d(centerFaceOffset, 0.0, new Rotation2d(Math.PI)));
      centerFaces[4] =
          aprilTagLayout
              .getTagPose(22)
              .get()
              .toPose2d()
              .plus(new Transform2d(centerFaceOffset, 0.0, new Rotation2d(Math.PI)));
      centerFaces[5] =
          aprilTagLayout
              .getTagPose(17)
              .get()
              .toPose2d()
              .plus(new Transform2d(centerFaceOffset, 0.0, new Rotation2d(Math.PI)));

      // Initialize algae heights
      AlgaeLevel[0] = SuperStructureState.INTAKE_ALGAE_L3;
      AlgaeLevel[1] = SuperStructureState.INTAKE_ALGAE_L2;
      AlgaeLevel[2] = SuperStructureState.INTAKE_ALGAE_L3;
      AlgaeLevel[3] = SuperStructureState.INTAKE_ALGAE_L2;
      AlgaeLevel[4] = SuperStructureState.INTAKE_ALGAE_L3;
      AlgaeLevel[5] = SuperStructureState.INTAKE_ALGAE_L2;

      // Initialize branch positions
      for (int face = 0; face < 6; face++) {
        Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
        Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
        for (var level : ReefLevel.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(
                              new Transform2d(adjustX - level.offset, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(
                              new Transform2d(adjustX - level.offset, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().rotateBy(new Rotation2d(Math.PI)).getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(
                              new Transform2d(adjustX - level.offset, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(
                              new Transform2d(adjustX - level.offset, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().rotateBy(new Rotation2d(Math.PI)).getRadians())));
        }
        branchPositions.add(fillRight);
        branchPositions.add(fillLeft);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final double separation = Units.inchesToMeters(72.0);
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), fieldWidth / 2.0, new Rotation2d());
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), middleIceCream.getY() + separation, new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), middleIceCream.getY() - separation, new Rotation2d());
  }

  @RequiredArgsConstructor
  public enum ReefLevel {
    L1(Units.inchesToMeters(25.0), 0, -0.6),
    L2(Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35, -0.65),
    L3(Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35, -0.65),
    L4(Units.inchesToMeters(72), -90, -0.6);

    public static ReefLevel fromLevel(int level) {
      return Arrays.stream(values())
          .filter(height -> height.ordinal() == level)
          .findFirst()
          .orElse(L4);
    }

    public static ReefLevel fromHeight(double height) {
      return Arrays.stream(values()).filter(level -> level.height == height).findFirst().orElse(L4);
    }

    public final double height;
    public final double pitch;
    public final double offset;
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final int aprilTagCount = 22;
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  @Getter
  public enum AprilTagLayoutType {
    OFFICIAL("2025-official"),
    NO_BARGE("2025-no-barge"),
    BLUE_REEF("2025-blue-reef"),
    RED_REEF("2025-red-reef");

    AprilTagLayoutType(String name) {

      try {
        layout =
            new AprilTagFieldLayout(
                Path.of(
                    Filesystem.getDeployDirectory().getPath(),
                    "apriltags",
                    fieldType.getJsonFolder(),
                    name + ".json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      if (layout == null) {
        layoutString = "";
      } else {
        try {
          layoutString = new ObjectMapper().writeValueAsString(layout);
        } catch (JsonProcessingException e) {
          throw new RuntimeException(
              "Failed to serialize AprilTag layout JSON " + toString() + "for Northstar");
        }
      }
    }

    private final AprilTagFieldLayout layout;
    private final String layoutString;
  }

  public record CoralObjective(int branchId, ReefLevel reefLevel) {}

  public record AlgaeObjective(int id) {}

  @RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    @Getter private final String jsonFolder;
  }
}
