package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PlaceCoralCommand;
import frc.robot.commands.SuperStructureCommand;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants.ReefLevel;
import java.util.Set;
import java.util.function.Supplier;

public class AutoScoreCoralAtBranchCommand extends SequentialCommandGroup {
  private ReefLevel reefLevel;
  private SuperStructureState superStructureState;

  /**
   * A command that scores a coral at a branch. Moves to the correct branch, moves the
   * superstructure to the correct height, and scores the coral.
   */
  public AutoScoreCoralAtBranchCommand(
      Drive drive,
      SuperStructure superStructure,
      EndEffector endEffector,
      Supplier<Pose3d> targetPose,
      Supplier<Integer> branchID) {

    addCommands(
        new InstantCommand(
            () -> {
              // set up starting state
              reefLevel = ReefLevel.fromHeight(targetPose.get().getZ());
              superStructureState =
                  switch (reefLevel) {
                    case L1 -> SuperStructureState.SCORE_CORAL_L1;
                    case L2 -> SuperStructureState.SCORE_CORAL_L2;
                    case L3 -> SuperStructureState.SCORE_CORAL_L3;
                    case L4 -> SuperStructureState.SCORE_CORAL_L4;
                    default -> SuperStructureState.SCORE_CORAL_L4;
                  };
              LEDSubsystem.getInstance().setStates(LEDStates.SCORING_LINE_UP);
            }),
        // move superstructure to state and PID to target pose
        // needs to be deferred so that ReefLevel gets set
        new DeferredCommand(
            () ->
                new ParallelCommandGroup(
                        new SuperStructureCommand(superStructure, () -> superStructureState),
                        DriveCommands.pidToPose(drive, () -> targetPose.get().toPose2d(), branchID)
                            .unless(() -> DriverStation.isTest()))
                    .andThen(
                        // place coral
                        new PlaceCoralCommand(reefLevel, superStructure, endEffector)),
            Set.of(superStructure)));
  }
}
