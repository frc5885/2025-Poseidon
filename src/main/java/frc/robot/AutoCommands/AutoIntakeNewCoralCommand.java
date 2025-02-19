package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.SuperStructureCommand;
import frc.robot.subsystems.Collector.Collector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.photon.Vision;
import frc.robot.util.TunableDouble;

public class AutoIntakeNewCoralCommand extends SequentialCommandGroup {
  double driveSpeed = -0.6;

  public AutoIntakeNewCoralCommand(
      Drive drive,
      SuperStructure superStructure,
      Collector collector,
      Vision vision,
      Pose2d intakePose) {
    addCommands(
        new ParallelCommandGroup(
            new DriveToPoseCommand(drive, () -> intakePose),
            new SuperStructureCommand(superStructure, SuperStructureState.INTAKE_CORAL)),
        new ParallelDeadlineGroup(
            new IntakeCoralCommand(collector),
            DriveCommands.driveToGamePiece(
                drive,
                TunableDouble.register("Drive/AimingSpeed", driveSpeed),
                () -> 0.0,
                () -> vision.getTargetX(2).getRadians())));
  }
}
