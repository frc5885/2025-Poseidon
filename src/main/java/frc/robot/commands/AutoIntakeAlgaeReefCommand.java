// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CalculateAlgaeStateUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.GamePieces.GamePieceVisualizer;
import java.util.Set;
import java.util.function.Supplier;

public class AutoIntakeAlgaeReefCommand extends SequentialCommandGroup {
  private final double kTransitionDistance = 0.3;
  private final double kDriveInSpeed = 1.0;
  private Pose2d targetPose;
  private Pose2d grabPose;
  private Pose2d transitionPose2d;
  private Supplier<SuperStructureState> stateSupplier;

  /**
   * A command that intakes algae off the reef. Moves to the closest face, moves the superstructure
   * to the correct height, and intakes the algae.
   */
  public AutoIntakeAlgaeReefCommand(
      Drive drive, SuperStructure superStructure, EndEffector endEffector) {

    addCommands(
        // Fake initializer to calculate target pose and state
        new InstantCommand(
            () -> {
              targetPose = drive.getPose().nearest(FieldConstants.getReefFaces());
              transitionPose2d =
                  targetPose.transformBy(
                      new Transform2d(-kTransitionDistance, 0.0, new Rotation2d()));
              stateSupplier = () -> CalculateAlgaeStateUtil.calculateIntakeState(targetPose);

              grabPose =
                  transitionPose2d.transformBy(
                      new Transform2d(kDriveInSpeed, 0.0, new Rotation2d()));

              LEDSubsystem.getInstance().setStates(LEDStates.ALGAE_INTAKE_LINE_UP);
            }),
        // go to transition pose
        new ParallelCommandGroup(
            new SuperStructureCommand(superStructure, () -> stateSupplier.get()),
            new DeferredCommand(
                    () -> DriveCommands.pidToPoseLooseTolerance(drive, () -> transitionPose2d),
                    Set.of(drive))
                .unless(() -> DriverStation.isTest())),
        // drive in to reef
        new ParallelDeadlineGroup(
            DriveCommands.driveStraight(drive, kDriveInSpeed)
                .until(() -> endEffector.isHoldingAlgae())
                .unless(() -> DriverStation.isTest())
                .withTimeout(Constants.kCurrentMode == Mode.SIM ? 1.0 : 30),
            new IntakeAlgaeCommand(endEffector)),
        new InstantCommand(
            () -> {
              GamePieceVisualizer.setHasAlgae(true);
            }),
        // raise elevator
        new SuperStructureCommand(
            superStructure,
            () -> CalculateAlgaeStateUtil.calculateAfterIntakeState(stateSupplier.get())),
        // back out of reef
        DriveCommands.driveStraight(drive, -kDriveInSpeed)
            .withTimeout(1.0)
            .unless(() -> DriverStation.isTest()));

    addRequirements(drive, superStructure);
  }
}
