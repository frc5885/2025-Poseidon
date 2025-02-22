package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class WaitUntilFarFromCommand extends Command {
  private final Supplier<Pose2d> m_drivePoseSupplier;
  private final double m_minDistance;
  private Pose2d m_startingPose;

  /**
   * A command that waits until the robot is a certain distance away from a its starting pose (when
   * the command was initialized).
   */
  public WaitUntilFarFromCommand(Supplier<Pose2d> drivePoseSupplier, double minDistance) {
    m_drivePoseSupplier = drivePoseSupplier;
    m_minDistance = minDistance;
  }

  @Override
  public void initialize() {
    m_startingPose = m_drivePoseSupplier.get();
  }

  @Override
  public boolean isFinished() {
    double currentDistance =
        m_drivePoseSupplier.get().getTranslation().getDistance(m_startingPose.getTranslation());
    return currentDistance >= m_minDistance;
  }
}
