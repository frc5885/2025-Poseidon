// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.ResetSuperStructureCommand;
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
  private Pose2d intakePose = new Pose2d(1.22, 0.88, Rotation2d.fromDegrees(54.27));
  private Pose2d initialPose = new Pose2d(7.3, 1.6, Rotation2d.fromRadians(Math.PI));

  /** Creates a new RightAuto. */
  public RightAuto(
      Drive drive, SuperStructure superStructure, EndEffector endEffector, Vision vision) {

    addCommands(
        // setup for sim
        new InstantCommand(
            () -> {
              intakePose = AllianceFlipUtil.apply(intakePose);
              if (Constants.kCurrentMode.equals(Mode.SIM)) {
                drive.setPose(AllianceFlipUtil.apply(initialPose));
              }
            }),

        // score first coral
        new AutoScoreCoralAtBranchCommand(
            drive,
            superStructure,
            endEffector,
            () -> FieldConstants.Reef.branchPositions.get(9).get(ReefLevel.L4)),
        // go intake
        new ParallelCommandGroup(
            new ResetSuperStructureCommand(drive, superStructure, false),
            drive.getDriveToPoseCommand(() -> intakePose, false)),

        // score second coral
        new AutoScoreCoralAtBranchCommand(
            drive,
            superStructure,
            endEffector,
            () -> FieldConstants.Reef.branchPositions.get(10).get(ReefLevel.L4)),
        // go intake
        new ParallelCommandGroup(
            new ResetSuperStructureCommand(drive, superStructure, false),
            drive.getDriveToPoseCommand(() -> intakePose, false)),

        // score third coral
        new AutoScoreCoralAtBranchCommand(
            drive,
            superStructure,
            endEffector,
            () -> FieldConstants.Reef.branchPositions.get(11).get(ReefLevel.L4)),
        // go intake
        new ParallelCommandGroup(
            new ResetSuperStructureCommand(drive, superStructure, false),
            drive.getDriveToPoseCommand(() -> intakePose, false)),

        // score fourth coral
        new AutoScoreCoralAtBranchCommand(
            drive,
            superStructure,
            endEffector,
            () -> FieldConstants.Reef.branchPositions.get(0).get(ReefLevel.L4)));
  }
}
