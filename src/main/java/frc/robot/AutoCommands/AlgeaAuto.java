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
import frc.robot.commands.AutoIntakeAlgaeReefCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreAlgaeNetCommand;
import frc.robot.commands.SuperStructureCommand;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.Side;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgeaAuto extends SequentialCommandGroup {
  /** Creates a new AlgeaAuto. */
  public AlgeaAuto(Drive drive, SuperStructure superStructure, EndEffector endEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(
            () -> {
              if (Constants.kCurrentMode.equals(Mode.SIM)) {
                new SuperStructureCommand(superStructure, () -> SuperStructureState.IDLE);
                drive.setPose(FieldConstants.getSimInitialPose(Side.MIDDLE));
              }
            }));
    addCommands(
        new AutoScoreCoralAtBranchCommand(
            drive, superStructure, endEffector, () -> 7, () -> 4, () -> false),
        new AutoIntakeAlgaeReefCommand(drive, superStructure, endEffector, () -> false),
        DriveCommands.pidToPose(
            drive, () -> AllianceFlipUtil.apply(new Pose2d(7.64, 5.19, new Rotation2d(Math.PI)))),
        new ScoreAlgaeNetCommand(drive, superStructure, endEffector));
  }
}
