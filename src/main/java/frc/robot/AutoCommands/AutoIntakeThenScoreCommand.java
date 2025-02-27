// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.SuperStructureCommand;
import frc.robot.commands.WaitUntilFarFromCommand;
import frc.robot.subsystems.Collector.Collector;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.vision.photon.Vision;
import frc.robot.util.FieldConstants.ReefLevel;
import frc.robot.util.TunableDouble;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeThenScoreCommand extends SequentialCommandGroup {
  private double kDistanceBeforeLowerSuperStructure = 0.25;
  private double kDriveSpeed = -0.6;

  private final double kTransitionDistance = 0.3;
  private Pose2d transitionPose2d;
  private ReefLevel reefLevel;
  private SuperStructureState superStructureState;

  /** Creates a new AutoIntakeThenScoreCommand. */
  public AutoIntakeThenScoreCommand(
      Drive drive,
      SuperStructure superStructure,
      Collector collector,
      EndEffector endEffector,
      Vision vision,
      Pose2d startIntakingPose,
      Supplier<Pose3d> targetPose) {
    addCommands(
        new InstantCommand(
            () -> {
              reefLevel = ReefLevel.fromHeight(targetPose.get().getZ());
              superStructureState =
                  switch (reefLevel) {
                    case L1 -> SuperStructureState.SCORE_CORAL_L1;
                    case L2 -> SuperStructureState.SCORE_CORAL_L2;
                    case L3 -> SuperStructureState.SCORE_CORAL_L3;
                    default -> SuperStructureState.SCORE_CORAL_L4;
                  };
              transitionPose2d =
                  targetPose
                      .get()
                      .toPose2d()
                      .transformBy(new Transform2d(-kTransitionDistance, 0.0, new Rotation2d()));
            }),
        new ParallelCommandGroup(
            new DriveToPoseCommand(
                drive,
                () -> startIntakingPose,
                DriveConstants.kDistanceTolerance,
                DriveConstants.kRotationTolerance,
                false),
            new WaitUntilFarFromCommand(drive::getPose, kDistanceBeforeLowerSuperStructure)
                .andThen(
                    new SuperStructureCommand(
                        superStructure, () -> SuperStructureState.INTAKE_CORAL))),
        new ParallelCommandGroup(
            DriveCommands.driveToGamePiece(
                    drive,
                    TunableDouble.register("Drive/AimingSpeed", kDriveSpeed),
                    () -> 0.0,
                    () -> 0.0,
                    () -> vision.getTargetX(2).getRadians(),
                    false)
                .until(collector::isCollected)
                .andThen(
                    new DriveToPoseCommand(
                            drive,
                            () -> transitionPose2d,
                            DriveConstants.kDistanceTolerance,
                            DriveConstants.kRotationTolerance,
                            false)
                        .unless(() -> DriverStation.isTest())),
            new IntakeCoralCommand(collector, endEffector)
                .andThen(new SuperStructureCommand(superStructure, () -> superStructureState))),
        DriveCommands.preciseChassisAlign(drive, () -> targetPose.get().toPose2d())
            .unless(() -> DriverStation.isTest()),
        new ScoreCoralCommand(endEffector));
  }
}
