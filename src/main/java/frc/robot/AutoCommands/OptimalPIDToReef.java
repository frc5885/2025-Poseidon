// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OptimalPIDToReef extends Command {
  private Drive m_drive;
  private Supplier<Pose2d> m_targetPoseSupplier;
  private Supplier<Integer> m_branchID;
  private Pose2d m_pathPlannerSetpoint;
  private Pose2d m_futurePose;
  private Command m_basicPIDToPoseCmd;
  private boolean m_isRunning;

  /** Creates a new OptimalPIDToPose. */
  public OptimalPIDToReef(
      Drive drive, Supplier<Pose2d> targetPoseSupplier, Supplier<Integer> branchID) {
    m_drive = drive;
    m_targetPoseSupplier = targetPoseSupplier;
    m_branchID = branchID;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isRunning = false;
    m_pathPlannerSetpoint = m_drive.getPathPlannerSetpoint();
    m_basicPIDToPoseCmd = DriveCommands.pidToPose(m_drive, m_targetPoseSupplier, m_branchID);

    m_futurePose =
        DriveCommands.calculateLookAheadPose(m_drive.getPose(), m_drive.getChassisSpeeds(), 0.25);
    Logger.recordOutput("Odometry/LookAheadPose", m_futurePose);
    if (m_futurePose.getTranslation().getDistance(m_targetPoseSupplier.get().getTranslation())
        < 0.55) {
      m_drive.setPathPlannerSetpoint(m_targetPoseSupplier.get());
    } else {
      m_drive
          .getBetterDriveToPoseCommand(() -> m_futurePose, m_targetPoseSupplier, true)
          .schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_isRunning) {
      if (m_drive.getPathPlannerSetpoint() != m_pathPlannerSetpoint
          || m_drive
                  .getPose()
                  .getTranslation()
                  .getDistance(m_targetPoseSupplier.get().getTranslation())
              < 0.55
          || m_futurePose.getTranslation().getDistance(m_targetPoseSupplier.get().getTranslation())
              < 0.55) {
        m_isRunning = true;
        m_basicPIDToPoseCmd.initialize();
      }

    } else {
      DriveCommands.getChassisController()
          .setGoalPose(
              new Pose2d(
                  m_drive.getPathPlannerSetpoint().getTranslation(),
                  DriveCommands.getChassisController().getFinalGoalPose().getRotation()));
      m_basicPIDToPoseCmd.execute();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.resetPathPlannerGetPose();
    m_basicPIDToPoseCmd.end(interrupted);
    m_isRunning = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_basicPIDToPoseCmd.isFinished() && m_isRunning;
  }
}
