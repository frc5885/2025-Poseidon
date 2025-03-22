package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class WaitUntilCloseToCommand extends Command {
  private final Supplier<Pose2d> m_drivePoseSupplier;
  private final Supplier<Pose2d> m_targetPoseSupplier;
  private final double m_minDistance;

  /** A command that waits until the robot is within a certain distance of a target pose. */
  public WaitUntilCloseToCommand(
      Supplier<Pose2d> drivePoseSupplier, Supplier<Pose2d> targetPoseSupplier, double minDistance) {
    m_drivePoseSupplier = drivePoseSupplier;
    m_targetPoseSupplier = targetPoseSupplier;
    m_minDistance = minDistance;
  }

  @Override
  public boolean isFinished() {
    double currentDistance =
        m_drivePoseSupplier
            .get()
            .getTranslation()
            .getDistance(m_targetPoseSupplier.get().getTranslation());
    Logger.recordOutput("DistanceForLED", currentDistance);
    return currentDistance <= m_minDistance;
  }
}
