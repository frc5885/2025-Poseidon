package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.SuperStructureCommand;
import frc.robot.subsystems.Collector.Collector;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.FieldConstants.ReefLevel;

public class AutoScoreCoralAtBranchCommand extends SequentialCommandGroup {

  /**
   * A command that scores a coral at a branch. Moves to the correct branch, moves the
   * superstructure to the correct height, and scores the coral.
   */
  public AutoScoreCoralAtBranchCommand(
      Drive drive,
      SuperStructure superStructure,
      EndEffector endEffector,
      Collector collector,
      Pose3d targetPose) {

    ReefLevel reefLevel = ReefLevel.fromHeight(targetPose.getZ());
    SuperStructureState superStructureState =
        switch (reefLevel) {
          case L1 -> SuperStructureState.SCORE_CORAL_L1;
          case L2 -> SuperStructureState.SCORE_CORAL_L2;
          case L3 -> SuperStructureState.SCORE_CORAL_L3;
          default -> SuperStructureState.SCORE_CORAL_L4;
        };

    Pose2d transitionPose2d =
        targetPose.toPose2d().transformBy(new Transform2d(-0.6, 0.0, new Rotation2d()));

    addCommands(
        new ParallelCommandGroup(
            new SuperStructureCommand(superStructure, superStructureState),
            new DriveToPoseCommand(
                drive,
                () -> targetPose.toPose2d(),
                DriveConstants.kDistanceTolerance,
                DriveConstants.kRotationTolerance)),
        new ScoreCoralCommand(endEffector, collector));
  }
}
