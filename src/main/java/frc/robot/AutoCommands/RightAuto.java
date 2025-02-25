// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.io.beambreak.BeamBreakIOSim;
import frc.robot.subsystems.Collector.Collector;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.photon.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightAuto extends SequentialCommandGroup {
  private Pose2d startIntakingPose = new Pose2d(2.6, 2.2, Rotation2d.fromRadians(0.85));
  private Pose2d initialPose = new Pose2d(7.3, 1.6, Rotation2d.fromRadians(Math.PI));
  /** Creates a new RightAuto. */
  public RightAuto(
      Drive drive,
      SuperStructure superStructure,
      EndEffector endEffector,
      Collector collector,
      Vision vision) {

    addCommands(
        // setup for sim
        new InstantCommand(
            () -> {
              startIntakingPose = AllianceFlipUtil.apply(startIntakingPose);
              if (Constants.kCurrentMode.equals(Mode.SIM)) {
                drive.setPose(AllianceFlipUtil.apply(initialPose));
                ((BeamBreakIOSim) endEffector.getCoralBeamBreakIO()).simulateGamePieceIntake(0.0);
              }
            }),

        // score first coral
        new AutoScoreCoralAtBranchCommand(
            drive,
            superStructure,
            endEffector,
            () -> FieldConstants.Reef.branchPositions.get(9).get(ReefLevel.L4)),
        new AutoIntakeNewCoralCommand(
            drive, superStructure, collector, endEffector, vision, startIntakingPose),

        // score second coral
        new AutoScoreCoralAtBranchCommand(
            drive,
            superStructure,
            endEffector,
            () -> FieldConstants.Reef.branchPositions.get(10).get(ReefLevel.L4)),
        new AutoIntakeNewCoralCommand(
            drive, superStructure, collector, endEffector, vision, startIntakingPose),

        // score third coral
        new AutoScoreCoralAtBranchCommand(
            drive,
            superStructure,
            endEffector,
            () -> FieldConstants.Reef.branchPositions.get(11).get(ReefLevel.L4)),
        new AutoIntakeNewCoralCommand(
            drive, superStructure, collector, endEffector, vision, startIntakingPose),

        // score fourth coral
        new AutoScoreCoralAtBranchCommand(
            drive,
            superStructure,
            endEffector,
            () -> FieldConstants.Reef.branchPositions.get(0).get(ReefLevel.L4)));
  }
}
