package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.util.FieldConstants.ReefLevel;

public class AutoScoreCoralAtBranchCommand extends SequentialCommandGroup {
  public AutoScoreCoralAtBranchCommand(
      Drive drive,
      SuperStructure superStructure,
      EndEffector endEffector,
      Collector collector,
      Pose3d targetPose) {

    ReefLevel reefLevel = ReefLevel.fromHeight(targetPose.getZ());
    SuperStructureState superStructureState;
    switch (reefLevel) {
      case L1:
        superStructureState = SuperStructureState.SCORE_CORAL_L1;
        break;
      case L2:
        superStructureState = SuperStructureState.SCORE_CORAL_L2;
        break;
      case L3:
        superStructureState = SuperStructureState.SCORE_CORAL_L3;
        break;
      default:
        superStructureState = SuperStructureState.SCORE_CORAL_L4;
        break;
    }

    addCommands(
        new ParallelCommandGroup(
            new DriveToPoseCommand(drive, () -> targetPose.toPose2d()),
            new SuperStructureCommand(superStructure, superStructureState)),
        new ScoreCoralCommand(endEffector, collector));
  }
}
