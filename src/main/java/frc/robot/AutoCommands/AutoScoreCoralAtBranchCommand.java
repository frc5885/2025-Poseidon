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
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.SuperStructureCommand;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.FieldConstants.ReefLevel;
import java.util.function.Supplier;

public class AutoScoreCoralAtBranchCommand extends SequentialCommandGroup {

  private final double kTransitionDistance = 0.3;

  private Pose2d transitionPose2d;
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
              LEDSubsystem.getInstance().setStates(LEDStates.SCORING_LINE_UP);
            }),
        new ParallelCommandGroup(
            new SuperStructureCommand(superStructure, () -> superStructureState),
            new DriveToPoseCommand(
                    drive,
                    () -> transitionPose2d,
                    DriveConstants.kDistanceTolerance,
                    DriveConstants.kRotationTolerance,
                    false)
                .unless(() -> DriverStation.isTest())),
        DriveCommands.preciseChassisAlign(drive, () -> targetPose.get().toPose2d())
            .unless(() -> DriverStation.isTest()),
        new ScoreCoralCommand(endEffector));
  }
}
