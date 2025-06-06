// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GamePieces.GamePieceVisualizer;
import java.util.Set;

public class ScoreAlgaeNetCommand extends SequentialCommandGroup {
  public ScoreAlgaeNetCommand(Drive drive, SuperStructure superStructure, EndEffector endEffector) {
    addCommands(
        new InstantCommand(
            () -> LEDSubsystem.getInstance().setStates(LEDSubsystem.LEDStates.SCORING_LINE_UP)),
        new ParallelCommandGroup(
            new DeferredCommand(
                    () ->
                        DriveCommands.pidToPoseLooseTolerance(
                            drive,
                            () ->
                                new Pose2d(
                                    AllianceFlipUtil.applyX(8.3),
                                    drive.getPose().getY(),
                                    AllianceFlipUtil.apply(Rotation2d.k180deg))),
                    Set.of(drive))
                .unless(() -> DriverStation.isTest()),
            new SuperStructureCommand(superStructure, () -> SuperStructureState.BEFORE_NET)),
        // exit the score command in simulation so that the visualizer works in sim
        new SequentialCommandGroup(
            // need to timeout if arm slams barge and can't hit setpoint
            new SuperStructureCommand(superStructure, () -> SuperStructureState.SCORE_ALGAE_NET)
                .withTimeout(0.7),
            new ScoreAlgaeCommand(endEffector)
                .withTimeout(
                    (Constants.kCurrentMode == Mode.SIM || DriverStation.isAutonomous())
                        ? 0.5
                        : 30)),
        new InstantCommand(() -> GamePieceVisualizer.setHasAlgae(false)));

    addRequirements(drive, superStructure);
  }
}
