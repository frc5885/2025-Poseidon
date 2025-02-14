// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.GamePieces;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class GamePieceVisualizer {
  private static final double ejectSpeed = 4; // Meters per sec
  private static Supplier<Pose2d> robotPoseSupplier = Pose2d::new;
  private static Supplier<Pose3d> endEffectorPoseSupplier = Pose3d::new;
  private static final List<Pose3d> fieldCoral = new ArrayList<>();
  private static final List<Pose3d> fieldAlgae = new ArrayList<>();
  private static boolean hasCoral = false;
  private static boolean hasAlgae = false;

  // Setters for static fields
  public static void setRobotPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
    GamePieceVisualizer.robotPoseSupplier = robotPoseSupplier;
  }

  public static void setEndEffectorPoseSupplier(Supplier<Pose3d> endEffectorPoseSupplier) {
    GamePieceVisualizer.endEffectorPoseSupplier = endEffectorPoseSupplier;
  }

  public static void setHasCoral(boolean hasCoral) {
    GamePieceVisualizer.hasCoral = hasCoral;
  }

  public static void setHasAlgae(boolean hasAlgae) {
    GamePieceVisualizer.hasAlgae = hasAlgae;
  }

  // Getters for static fields
  public static boolean hasCoral() {
    return hasCoral;
  }

  public static boolean hasAlgae() {
    return hasAlgae;
  }

  public static List<Pose3d> getFieldCoral() {
    // return all coral
    // result needs to be checked for nulls
    return fieldCoral;
  }

  public static List<Pose3d> getFieldAlgae() {
    // return all algae
    // result needs to be checked for nulls
    return fieldAlgae;
  }

  /** Show all field coral and algae */
  public static void showFieldGamePieces() {
    if (fieldCoral.isEmpty()) {
      Logger.recordOutput("GamePieceVisualizer/Coral/Field", new Pose3d[] {});
    }
    if (fieldAlgae.isEmpty()) {
      Logger.recordOutput("GamePieceVisualizer/Algae/Field", new Pose3d[] {});
    }
    Stream<Pose3d> presentCoral = fieldCoral.stream().filter(Objects::nonNull);
    Logger.recordOutput("GamePieceVisualizer/Coral/Field", presentCoral.toArray(Pose3d[]::new));

    Stream<Pose3d> presentAlgae = fieldAlgae.stream().filter(Objects::nonNull);
    Logger.recordOutput("GamePieceVisualizer/Algae/Field", presentAlgae.toArray(Pose3d[]::new));
  }

  public static void clearFieldGamePieces() {
    fieldCoral.clear();
    fieldAlgae.clear();
  }

  /** Add all notes to be shown at the beginning of auto */
  public static void resetFieldGamePieces() {
    clearFieldGamePieces();
    fieldCoral.addAll(
        CoralTargetModel.getCoralPositions().stream().filter(Objects::nonNull).toList());
    Logger.recordOutput("GamePieceVisualizer/Coral/Scored", new Pose3d[] {});
    Logger.recordOutput("GamePieceVisualizer/Algae/Scored", new Pose3d[] {});
  }

  /**
   * Take note from field notes
   *
   * @param note Index of note
   */
  // public static void takeFieldNote(int note) {
  //   fieldNotes.set(note, null);
  //   // refresh the notes shown on the field
  //   GamePieceVisualizer.showFieldNotes();
  // }

  /** Shows the currently held coral/algae if there is one */
  public static void showHeldGamePieces() {
    if (hasCoral) {
      Logger.recordOutput("GamePieceVisualizer/Coral/Held", new Pose3d[] {getCoralIndexerPose3d()});
    } else {
      Logger.recordOutput("GamePieceVisualizer/Coral/Held", new Pose3d[] {});
    }
    if (hasAlgae) {
      Logger.recordOutput("GamePieceVisualizer/Algae/Held", new Pose3d[] {getAlgaeIndexerPose3d()});
    } else {
      Logger.recordOutput("GamePieceVisualizer/Algae/Held", new Pose3d[] {});
    }
  }

  /** Shoots coral out and leaves it in front of the indexer */
  public static Command scoreCoral() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  hasCoral = false;
                  final Pose3d startPose = getCoralIndexerPose3d();
                  final Pose3d endPose =
                      startPose.transformBy(new Transform3d(0.5, 0, 0, new Rotation3d()));

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / ejectSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "GamePieceVisualizer/Coral/Scored",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput(
                                "GamePieceVisualizer/Coral/Scored", new Pose3d[] {});
                            fieldCoral.add(endPose);
                          });
                },
                Set.of())
            .ignoringDisable(true));
  }

  /** Shoots algae out and leaves it in front of the indexer */
  public static Command scoreAlgae() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  hasAlgae = false;
                  final Pose3d startPose = getAlgaeIndexerPose3d();
                  final Pose3d endPose =
                      startPose.transformBy(new Transform3d(0.5, 0, 0, new Rotation3d()));

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / ejectSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "GamePieceVisualizer/Algae/Scored",
                                  new Pose3d[] {
                                    startPose.interpolate(endPose, timer.get() / duration)
                                  }))
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput(
                                "GamePieceVisualizer/Algae/Scored", new Pose3d[] {});
                            fieldAlgae.add(endPose);
                          });
                },
                Set.of())
            .ignoringDisable(true));
  }

  /*
   * Returns the 3D pose of the coral indexer in field space for visualization.
   */
  private static Pose3d getCoralIndexerPose3d() {
    Transform3d indexerTransform =
        new Transform3d(
                endEffectorPoseSupplier.get().getTranslation(),
                endEffectorPoseSupplier.get().getRotation())
            .plus(new Transform3d(0.1, 0.0, 0.0, new Rotation3d()));
    return new Pose3d(robotPoseSupplier.get()).transformBy(indexerTransform);
  }

  /*
   * Returns the 3D pose of the algae indexer in field space for visualization.
   */
  private static Pose3d getAlgaeIndexerPose3d() {
    Transform3d indexerTransform =
        new Transform3d(
                endEffectorPoseSupplier.get().getTranslation(),
                endEffectorPoseSupplier.get().getRotation())
            .plus(
                new Transform3d(
                    0.07, 0.0, -0.37, new Rotation3d(0, Units.degreesToRadians(45), 0)));
    return new Pose3d(robotPoseSupplier.get()).transformBy(indexerTransform);
  }
}
