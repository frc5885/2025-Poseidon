// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.forbidden;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.PoseUtil;
import java.util.function.Supplier;
import java.util.logging.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MichaelCommand extends Command {
  /** Creates a new ReplaceMeCommand. */
  private final Drive m_drive;

  private final Supplier<Pose2d> m_targetPose;

  private static int factor = 0;

  private final NetworkTable m_networkTable;
  private final String NETWORK_ENTRY = "ReefTargets";
  private final String NETWORK_ENTRY_level = "ReefTargetsLevel";

  public MichaelCommand(Drive drive, Supplier<Pose2d> targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.]
    m_drive = drive;
    m_targetPose = targetPose;

    m_networkTable = NetworkTableInstance.getDefault().getTable("ReefPanel");
    m_networkTable.getDoubleTopic(NETWORK_ENTRY).publish();
    m_networkTable.getDoubleTopic(NETWORK_ENTRY_level).publish();

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Command cmd =
    //     DriveCommands.driveToPose(
    //         m_drive,
    //         () ->
    //             Constants.kBluePoses[(int)
    // m_networkTable.getEntry(NETWORK_ENTRY).getDouble(0.0)]);
    // cmd.schedule(); // Make sure the returned command actually runs
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static void increasePose() {
    // Increase the target pose
    factor = 1;
    System.out.println("Factor: " + factor);
    return;
  }

  public static void decreasePose() {
    // Decrease the target pose
    factor = -1;
  }

  // PoseUtil useless
  public int indexer(Pose2d[] poses, Pose2d target) {
    // set closest pose to first pose
    Pose2d closestPose = poses[0];
    int index = 0;

    // set closest distance to distance between first pose and target
    double closestDistance = target.getTranslation().getDistance(closestPose.getTranslation());
    for (int i = 1; i < poses.length; i++) {
      double distance = target.getTranslation().getDistance(poses[i].getTranslation());
      if (distance < closestDistance) {
        closestPose = poses[i];
        closestDistance = distance;
        Logger.getLogger(PoseUtil.class.getName()).info("Closest pose: " + closestPose);
        Logger.getLogger(PoseUtil.class.getName()).info("Pose its checking: " + poses[i]);
        index = i;
      }
    }
    System.out.println("Closest pose: " + closestPose);
    return index;
  }
}
