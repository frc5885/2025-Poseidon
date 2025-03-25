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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.WaitUntilCloseToCommand;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Feeder.FeederConstants.FeederState;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants.Reef;
import frc.robot.util.FieldConstants.ReefLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  private Pose2d intakePose = new Pose2d(1.22, 0.88, Rotation2d.fromDegrees(54.27));
  private double ledDistance = 1.0;
  /** Creates a new RightAuto. */
  public TestAuto(Drive drive, Feeder feeder) {

    addCommands(
        new InstantCommand(
            () -> {
              if (Constants.kCurrentMode == Mode.SIM) {
                drive.setPose(AllianceFlipUtil.apply(drive.getPose()));
              }
              feeder.runFeeder(8.0);
            }),
        DriveCommands.pidToPose(
            drive, () -> Reef.branchPositions.get(0).get(ReefLevel.L4).toPose2d()),
        new InstantCommand(() -> feeder.setFeederState(FeederState.FEEDING)),
        DriveCommands.auto_basicPathplannerToPose(drive, () -> intakePose, false)
            .alongWith(
                new WaitUntilCloseToCommand(() -> drive.getPose(), () -> intakePose, ledDistance)
                    .andThen(new InstantCommand(() -> LEDSubsystem.getInstance().flashGreen()))),
        DriveCommands.pidToPose(
            drive, () -> Reef.branchPositions.get(0).get(ReefLevel.L4).toPose2d()),
        new InstantCommand(() -> feeder.setFeederState(FeederState.FEEDING)),
        DriveCommands.auto_basicPathplannerToPose(drive, () -> intakePose, false)
            .alongWith(
                new WaitUntilCloseToCommand(() -> drive.getPose(), () -> intakePose, ledDistance)
                    .andThen(new InstantCommand(() -> LEDSubsystem.getInstance().flashGreen()))),
        DriveCommands.pidToPose(
            drive, () -> Reef.branchPositions.get(0).get(ReefLevel.L4).toPose2d()),
        new InstantCommand(() -> feeder.setFeederState(FeederState.FEEDING)),
        DriveCommands.auto_basicPathplannerToPose(drive, () -> intakePose, false)
            .alongWith(
                new WaitUntilCloseToCommand(() -> drive.getPose(), () -> intakePose, ledDistance)
                    .andThen(new InstantCommand(() -> LEDSubsystem.getInstance().flashGreen()))),
        DriveCommands.pidToPose(
            drive, () -> Reef.branchPositions.get(0).get(ReefLevel.L4).toPose2d()),
        new InstantCommand(() -> feeder.stop()));
  }
}
