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
import frc.robot.util.FieldConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.littletonrobotics.junction.Logger;

public class GamePieceVisualizer {
  private static final double ejectSpeed = 4; // Meters per sec
  private static Supplier<Pose2d> robotPoseSupplier = Pose2d::new;
  private static Supplier<Pose3d> endEffectorPoseSupplier = Pose3d::new;
  private static final List<Pose3d> scoredCoral = new ArrayList<>();
  private static final List<Pose3d> scoredAlgae = new ArrayList<>();
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

  public static void clearFieldGamePieces() {
    scoredCoral.clear();
    scoredAlgae.clear();
  }

  /** Add all coral to be shown at the beginning of auto */
  public static void resetFieldGamePieces() {
    clearFieldGamePieces();
    SimulatedArena.getInstance().resetFieldForAuto();
    Logger.recordOutput("GamePieceVisualizer/Coral/Scored", new Pose3d[] {});
    Logger.recordOutput("GamePieceVisualizer/Algae/Scored", new Pose3d[] {});

    // Add HP coral to the field
    CoralTargetModel.getCoralPositions().stream()
        .map(pose -> new ReefscapeCoralOnField(pose))
        .forEach(SimulatedArena.getInstance()::addGamePiece);
  }

  /**
   * Checks which of the 4 starting coral poses is missing, and respawns it using CoralTargetModel
   * generation.
   */
  public static void respawnCoral() {
    List<Pose2d> basePositions = CoralTargetModel.getCoralPositions();
    for (int i = 0; i < basePositions.size(); i++) {
      Pose2d coralPose = basePositions.get(i);
      boolean found =
          SimulatedArena.getInstance().getGamePiecesByType("Coral").stream()
              .anyMatch(
                  coral ->
                      new Translation2d(coral.getX(), coral.getY())
                                  .getDistance(coralPose.getTranslation())
                              < 2.0
                          && coral.getRotation().getY() == 0);
      if (!found) {
        // Generate a new candidate for the missing coral using the CoralTargetModel generation
        List<Pose2d> newPositions = CoralTargetModel.getCoralPositions();
        Pose2d newRespawnPose = newPositions.get(i);
        Commands.waitSeconds(3)
            .andThen(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePiece(new ReefscapeCoralOnField(newRespawnPose)))
            .schedule();
        break;
      }
    }
  }

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
                      startPose.transformBy(new Transform3d(-0.3, 0, 0, new Rotation3d()));

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / ejectSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  scoredCoral.add(startPose);
                  return Commands.run(
                          () -> {
                            scoredCoral.set(
                                scoredCoral.size() - 1,
                                startPose.interpolate(endPose, timer.get() / duration));
                            Logger.recordOutput(
                                "GamePieceVisualizer/Coral/Scored",
                                scoredCoral.toArray(Pose3d[]::new));
                          })
                      .until(() -> timer.hasElapsed(duration));
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
            .plus(
                new Transform3d(-0.05, 0.0, 0.0, new Rotation3d(0, Units.degreesToRadians(90), 0)));
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
            .plus(new Transform3d(0.1, 0.0, 0, new Rotation3d()));
    return new Pose3d(robotPoseSupplier.get()).transformBy(indexerTransform);
  }

  /**
   * Checks if the robot is near a loading station. This is used in simulation to determine when the
   * handoff should happen
   */
  public static boolean isNearLoadingStation() {
    Pose2d[] loadingStationPoses = FieldConstants.HumanPlayerStations;
    Pose2d robotPose = robotPoseSupplier.get();
    for (Pose2d loadingStationPose : loadingStationPoses) {
      if (robotPose
              .transformBy(new Transform2d(-0.2, 0, new Rotation2d()))
              .getTranslation()
              .getDistance(loadingStationPose.getTranslation())
          < 1.0) {
        Logger.recordOutput("GamePieceVisualizer/NearLoadingStation", true);
        return true;
      }
    }
    Logger.recordOutput("GamePieceVisualizer/NearLoadingStation", false);
    return false;
  }
}
