// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.CalculateAlgaeStateUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.GamePieces.GamePieceVisualizer;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.IntStream;

public class ManualIntakeAlgaeReefCommand extends SequentialCommandGroup {
  private final double kTransitionDistance = 0.4;
  private Pose2d targetPose;
  private Pose2d transitionPose2d;
  private Supplier<SuperStructureState> stateSupplier;

  /**
   * A command that intakes algae off the reef. Moves to the closest face, moves the superstructure
   * to the correct height, and intakes the algae.
   */
  public ManualIntakeAlgaeReefCommand(
      Drive drive,
      CommandXboxController controller,
      SuperStructure superStructure,
      EndEffector endEffector,
      Supplier<Pose2d> drivePose) {

    addCommands(
        // Fake initializer to calculate target pose and state
        new InstantCommand(
            () -> {
              List<Pose2d> faces =
                  List.of(
                      IntStream.range(0, FieldConstants.Reef.centerFaces.length)
                          .mapToObj(i -> AllianceFlipUtil.apply(FieldConstants.Reef.centerFaces[i]))
                          .toArray(Pose2d[]::new));
              targetPose = drivePose.get().nearest(faces);
              transitionPose2d =
                  targetPose.transformBy(
                      new Transform2d(-kTransitionDistance, 0.0, new Rotation2d()));
              stateSupplier = () -> CalculateAlgaeStateUtil.calculateIntakeState(targetPose);
              endEffector.runEndEffectorIntake();
              LEDSubsystem.getInstance().setStates(LEDStates.ALGAE_INTAKE_LINE_UP);
            }),
        new ParallelCommandGroup(
            new SuperStructureCommand(superStructure, () -> stateSupplier.get()),
            new InstantCommand(
                () -> {
                  GamePieceVisualizer.setHasAlgae(true);
                })));

    addRequirements(superStructure);
  }
}
