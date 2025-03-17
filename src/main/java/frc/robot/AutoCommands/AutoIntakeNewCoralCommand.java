// package frc.robot.AutoCommands;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.DriveCommands;
// import frc.robot.commands.DriveToPoseCommand;
// import frc.robot.commands.IntakeCoralCommand;
// import frc.robot.commands.SuperStructureCommand;
// import frc.robot.commands.WaitUntilFarFromCommand;
// import frc.robot.subsystems.Collector.Collector;
// import frc.robot.subsystems.EndEffector.EndEffector;
// import frc.robot.subsystems.Feeder.Feeder;
// import frc.robot.subsystems.SuperStructure.SuperStructure;
// import frc.robot.subsystems.SuperStructure.SuperStructureState;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.drive.DriveConstants;
// import frc.robot.subsystems.vision.photon.Vision;
// import frc.robot.util.TunableDouble;

// public class AutoIntakeNewCoralCommand extends SequentialCommandGroup {
//   private double kDriveSpeed = -0.6;
//   private double kDistanceBeforeLowerSuperStructure = 0.25;

//   /**
//    * A command that intakes a new coral. Moves the robot to a starting pose, lowers the
//    * superstructure, and intakes a coral using game piece tracking. Ends when the robot has
// intaked
//    * the coral.
//    */
//   public AutoIntakeNewCoralCommand(
//       Drive drive,
//       SuperStructure superStructure,
//       Feeder feeder,
//       EndEffector endEffector,
//       Vision vision,
//       Pose2d startIntakingPose) {

//     addCommands(
//         new ParallelCommandGroup(
//             new DriveToPoseCommand(
//                 drive,
//                 () -> startIntakingPose,
//                 DriveConstants.kDistanceTolerance,
//                 DriveConstants.kRotationTolerance,
//                 false),
//             new WaitUntilFarFromCommand(drive::getPose, kDistanceBeforeLowerSuperStructure)
//                 .andThen(
//                     new SuperStructureCommand(
//                         superStructure, () -> SuperStructureState.INTAKE_CORAL))),
//         new ParallelDeadlineGroup(
//             new IntakeCoralCommand(collector, endEffector),
//             DriveCommands.driveToGamePiece(
//                 drive,
//                 TunableDouble.register("Drive/AimingSpeed", kDriveSpeed),
//                 () -> 0.0,
//                 () -> 0.0,
//                 () -> vision.getTargetX(2).getRadians(),
//                 false)));
//   }
// }
