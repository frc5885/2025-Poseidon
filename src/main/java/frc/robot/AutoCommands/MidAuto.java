// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoIntakeAlgaeReefCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreAlgaeCommand;
import frc.robot.commands.SuperStructureCommand;
import frc.robot.commands.WaitUntilFarFromCommand;
import frc.robot.io.beambreak.BeamBreakIOSim;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.photon.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidAuto extends SequentialCommandGroup {

  private Pose2d initialPose = new Pose2d(7.3, 4.0, new Rotation2d(Math.PI));
  /** Creates a new MidAuto. */
  public MidAuto(
      Drive drive,
      SuperStructure superStructure,
      EndEffector endEffector,
      //   Collector collector,
      Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(
            () -> {
              if (Constants.kCurrentMode.equals(Mode.SIM)) {
                drive.setPose(AllianceFlipUtil.apply(initialPose));
                ((BeamBreakIOSim) endEffector.getCoralBeamBreakIO()).simulateGamePieceIntake(0.0);
              }
            }),
        new AutoIntakeAlgaeReefCommand(drive, superStructure, endEffector, () -> drive.getPose()),
        new WaitUntilFarFromCommand(() -> drive.getPose(), 0.45 + Units.inchesToMeters(7.0))
            .deadlineFor(
                new RunCommand(() -> drive.runVelocity(new ChassisSpeeds(-0.7, 0.0, 0.0)), drive)),
        // new AutoScoreCoralAtBranchCommand(
        //     drive,
        //     superStructure,
        //     endEffector,
        //     () -> FieldConstants.Reef.branchPositions.get(7).get(FieldConstants.ReefLevel.L4)),
        // new WaitUntilFarFromCommand(() -> drive.getPose(), 0.45 + Units.inchesToMeters(7.0))
        //     .deadlineFor(
        //         new RunCommand(() -> drive.runVelocity(new ChassisSpeeds(-1.0, 0.0, 0.0)),
        // drive)),

        // score algae processor
        new ParallelCommandGroup(
            new SuperStructureCommand(superStructure, () -> SuperStructureState.INTAKE_CORAL),
            drive.getDriveToPoseCommand(
                () ->
                    FieldConstants.Processor.centerFace.transformBy(
                        new Transform2d(-0.6, 0.0, new Rotation2d())),
                false)),
        new ParallelCommandGroup(
            new SuperStructureCommand(
                superStructure, () -> SuperStructureState.SCORE_ALGAE_PROCESSOR),
            DriveCommands.preciseChassisAlign(drive, () -> FieldConstants.Processor.centerFace)),
        new ScoreAlgaeCommand(endEffector),

        // intake next algae reef
        new AutoIntakeAlgaeReefCommand(drive, superStructure, endEffector, () -> drive.getPose()),
        new WaitUntilFarFromCommand(() -> drive.getPose(), 0.45 + Units.inchesToMeters(7.0))
            .deadlineFor(
                new RunCommand(() -> drive.runVelocity(new ChassisSpeeds(-0.7, 0.0, 0.0)), drive)),

        // score next algae processor
        new ParallelCommandGroup(
            new SuperStructureCommand(
                superStructure, () -> SuperStructureState.SCORE_ALGAE_PROCESSOR),
            drive.getDriveToPoseCommand(
                () ->
                    FieldConstants.Processor.centerFace.transformBy(
                        new Transform2d(-0.6, 0.0, new Rotation2d())),
                false)),
        DriveCommands.preciseChassisAlign(drive, () -> FieldConstants.Processor.centerFace),
        new ScoreAlgaeCommand(endEffector));
  }
}
