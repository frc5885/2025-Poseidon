// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoIntakeAlgaeReefCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreAlgaeNetCommand;
import frc.robot.commands.SuperStructureCommand;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BargeAuto extends SequentialCommandGroup {

  /** Creates a new BargeAuto. */
  public BargeAuto(
      Drive drive, SuperStructure superStructure, Feeder feeder, EndEffector endEffector) {
    // Sim setup
    addCommands(
        new InstantCommand(
            () -> {
              if (Constants.kCurrentMode.equals(Mode.SIM)) {
                drive.setPose(FieldConstants.getSimInitialMiddlePose());
                feeder.simulateCoralFeed();
              }
            }));

    // start intake immediately
    addCommands(new InstantCommand(() -> endEffector.runEndEffectorIntake()));

    // score first coral
    int branchNum = 6;
    addCommands(
        new AutoScoreCoralAtBranchCommand(
            drive,
            superStructure,
            endEffector,
            () -> branchNum,
            () -> ReefLevel.L4.ordinal(),
            () -> false));

    // drive back a bit
    addCommands(DriveCommands.driveStraight(drive, -1.0).withTimeout(0.5));

    // intake algae
    addCommands(new AutoIntakeAlgaeReefCommand(drive, superStructure, endEffector, () -> false));

    // drive to barge pose
    addCommands(
        DriveCommands.auto_basicPathplannerToPose(
            drive, () -> FieldConstants.getAutonomousAlgaeScorePose()));

    // score algae
    addCommands(new ScoreAlgaeNetCommand(drive, superStructure, endEffector));

    // move to next algae
    addCommands(
        new SuperStructureCommand(superStructure, () -> SuperStructureState.INTAKE_ALGAE_L3)
            .alongWith(
                DriveCommands.auto_basicPathplannerToPose(
                    drive, () -> FieldConstants.getAutonomous2ndAlgaeLineupPose())));

    // intake algae
    addCommands(new AutoIntakeAlgaeReefCommand(drive, superStructure, endEffector, () -> false));

    // score algae
    addCommands(new ScoreAlgaeNetCommand(drive, superStructure, endEffector));
  }
}
