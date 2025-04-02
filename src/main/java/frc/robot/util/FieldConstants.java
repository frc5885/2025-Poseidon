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
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import java.io.IOException;
import java.nio.file.Path;
import java.util.*;
import java.util.stream.Collectors;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final FieldType fieldType;
  public static final double fieldLength;
  public static final double fieldWidth;

  // Static initializer block
  static {
    fieldType = FieldType.WELDED;

    // Load field dimensions from the layout
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    fieldLength = layout.getFieldLength();
    fieldWidth = layout.getFieldWidth();
  }

  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line
  public static final double algaeDiameter = Units.inchesToMeters(16);

  public static class Processor {
    public static final Pose2d centerFace;

    static {
      // Load layout directly to avoid circular dependency
      AprilTagFieldLayout aprilTagLayout;
      try {
        aprilTagLayout =
            new AprilTagFieldLayout(
                Path.of(
                    Filesystem.getDeployDirectory().getPath(),
                    "apriltags",
                    fieldType.getJsonFolder(),
                    "2025-official.json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

      centerFace =
          aprilTagLayout
              .getTagPose(16)
              .get()
              .toPose2d()
              .plus(new Transform2d(0.9, 0.0, new Rotation2d(Math.PI)));
    }
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

    public static final Pose2d[] centerFaces;
    public static final SuperStructureState[] AlgaeLevel;
    public static final double centerFaceOffset = 0.65;
    public static final List<Map<ReefLevel, Pose3d>> branchPositions = new ArrayList<>();

    static {
      centerFaces = new Pose2d[6];
      AlgaeLevel = new SuperStructureState[6];

      // Initialize faces - load layout directly to avoid circular dependency
      AprilTagFieldLayout aprilTagLayout;
      try {
        aprilTagLayout =
            new AprilTagFieldLayout(
                Path.of(
                    Filesystem.getDeployDirectory().getPath(),
                    "apriltags",
                    fieldType.getJsonFolder(),
                    "2025-official.json"));
      } catch (IOException e) {
        throw new RuntimeException(e);
      }

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
          double adjustY = Units.inchesToMeters(6.0);

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(
                              new Transform2d(
                                  adjustX - level.xOffset,
                                  adjustY - level.yOffset,
                                  new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(
                              new Transform2d(
                                  adjustX - level.xOffset,
                                  adjustY - level.yOffset,
                                  new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection
                          .getRotation()
                          .rotateBy(
                              new Rotation2d(
                                  Math.PI + Units.degreesToRadians(level.rotationOffset)))
                          .getRadians())));
          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(
                              new Transform2d(
                                  adjustX - level.xOffset,
                                  -adjustY + level.yOffset,
                                  new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(
                              new Transform2d(
                                  adjustX - level.xOffset,
                                  -adjustY + level.yOffset,
                                  new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection
                          .getRotation()
                          .rotateBy(
                              new Rotation2d(
                                  Math.PI - Units.degreesToRadians(level.rotationOffset)))
                          .getRadians())));
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
    // height, pitch, offset (x, y, rotationDegrees)
    L1(Units.inchesToMeters(25.0), 0, -0.57, Units.inchesToMeters(5.0), 0.0),
    L2(Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35, -0.57, 0.0, 0.0),
    L3(Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35, -0.75, 0.0, 0.0),
    L4(Units.inchesToMeters(72), -90, -0.78, 0.0, 0.0);

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
    public final double xOffset;
    public final double yOffset;
    public final double rotationOffset; // in degrees
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final int aprilTagCount = 22;
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  public static Pose2d[] HumanPlayerStations;

  static {
    // Initialize HumanPlayerStations directly to avoid circular dependency
    AprilTagFieldLayout officialLayout;
    try {
      officialLayout =
          new AprilTagFieldLayout(
              Path.of(
                  Filesystem.getDeployDirectory().getPath(),
                  "apriltags",
                  fieldType.getJsonFolder(),
                  "2025-official.json"));
    } catch (IOException e) {
      throw new RuntimeException(e);
    }

    HumanPlayerStations =
        new Pose2d[] {
          officialLayout.getTagPose(1).get().toPose2d(),
          officialLayout.getTagPose(2).get().toPose2d(),
          officialLayout.getTagPose(12).get().toPose2d(),
          officialLayout.getTagPose(13).get().toPose2d()
        };
  }

  @Getter
  public enum AprilTagLayoutType {
    NO_BARGE("2025-no-barge"),
    BLUE_REEF("2025-blue-reef"),
    RED_REEF("2025-red-reef"),
    OFFICIAL("2025-official"),
    REEFS("2025-both-reefs");

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

  public static enum Side {
    LEFT,
    RIGHT,
    MIDDLE;

    public static Side from(int value) {
      return value == 0 ? RIGHT : LEFT;
    }
  }

  public record CoralObjective(int branchId, ReefLevel reefLevel) {}

  public record AlgaeObjective(int id) {}

  @RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    @Getter private final String jsonFolder;
  }

  public static Pose3d getBranchPose3d(int id, ReefLevel level) {
    return AllianceFlipUtil.apply(Reef.branchPositions.get(id).get(level));
  }

  public static Pose2d getBranchPose2d(int id, ReefLevel level) {
    return getBranchPose3d(id, level).toPose2d();
  }

  public static Pose3d getBranchPose3d(int face, ReefLevel level, Side side) {
    return AllianceFlipUtil.apply(
        Reef.branchPositions.get(face * 2 + (side == Side.RIGHT ? 0 : 1)).get(level));
  }

  public static Pose2d getBranchPose2d(int face, ReefLevel level, Side side) {
    return getBranchPose3d(face, level, side).toPose2d();
  }

  public static int getBranchID(int face, Side side) {
    return face * 2 + (side == Side.RIGHT ? 0 : 1);
  }

  public static ArrayList<Pose2d> getReefFaces() {
    return Arrays.stream(Reef.centerFaces)
        .map(AllianceFlipUtil::apply)
        .collect(Collectors.toCollection(ArrayList::new));
  }

  public static Translation2d getReefCenter() {
    return AllianceFlipUtil.apply(Reef.center);
  }

  public static ArrayList<Pose3d> getAllL4Poses() {
    return Reef.branchPositions.stream()
        .map(map -> AllianceFlipUtil.apply(map.get(ReefLevel.L4)))
        .collect(Collectors.toCollection(ArrayList::new));
  }

  public static Pose2d getIntakePose(Side side) {
    boolean isRed = AllianceFlipUtil.shouldFlip();
    Pose2d pose = new Pose2d();

    if (!isRed && side == Side.RIGHT) {
      pose = new Pose2d(1.25, 1.0, Rotation2d.fromDegrees(54.5));
    } else if (!isRed && side == Side.LEFT) {
      pose = new Pose2d(1.25, 7.0, Rotation2d.fromDegrees(-54.5));
    } else if (isRed && side == Side.RIGHT) {
      pose = new Pose2d(16.30, 7.0, Rotation2d.fromDegrees(-125.5));
    } else if (isRed && side == Side.LEFT) {
      pose = new Pose2d(16.30, 1.0, Rotation2d.fromDegrees(125.5));
    }

    return pose;
  }

  public static Pose2d getSimInitialPose(Side side) {
    boolean isRed = AllianceFlipUtil.shouldFlip();
    Pose2d pose = new Pose2d();

    if (!isRed && side == Side.RIGHT) {
      pose = new Pose2d(7.3, 1.6, Rotation2d.fromRadians(Math.PI));
    } else if (!isRed && side == Side.LEFT) {
      pose = new Pose2d(7.3, 6.45, Rotation2d.fromRadians(Math.PI));
    } else if (isRed && side == Side.RIGHT) {
      pose = new Pose2d(10.25, 6.45, Rotation2d.fromRadians(0));
    } else if (isRed && side == Side.LEFT) {
      pose = new Pose2d(10.25, 1.6, Rotation2d.fromRadians(0));
    } else if (!isRed && side == Side.MIDDLE) {
      pose = new Pose2d(7.6, 4.0, new Rotation2d(Math.PI));
    } else if (isRed && side == Side.MIDDLE) {
      pose = new Pose2d(10, 4.0, new Rotation2d());
    }

    return pose;
  }

  public static Pose2d getSimInitialMiddlePose() {
    return AllianceFlipUtil.apply(new Pose2d(7.3, 4.6, Rotation2d.fromRadians(Math.PI)));
  }

  public static Pose2d getAutonomousAlgaeScorePose() {
    return AllianceFlipUtil.apply(new Pose2d(8.0, 5.0, Rotation2d.fromRadians(Math.PI)));
  }

  public static Pose2d getAutonomous2ndAlgaeLineupPose() {
    return AllianceFlipUtil.apply(new Pose2d(5.9, 5.5, Rotation2d.fromRadians(-2.0)));
  }

  public static Rotation2d getProcessorAngle() {
    return AllianceFlipUtil.apply(new Rotation2d(3 * Math.PI / 2));
  }
}
