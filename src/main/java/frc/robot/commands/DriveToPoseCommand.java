// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveToPoseCommand extends Command {
  private final Drive m_drive;
  private Supplier<Pose2d> m_targetPose;
  private Command m_command;

  public DriveToPoseCommand(Drive drive, Supplier<Pose2d> targetPose) {
    m_drive = drive;
    m_targetPose = targetPose;
    m_command = m_drive.getDriveToPoseCommand(m_targetPose);

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    m_command.initialize();
  }

  @Override
  public void execute() {
    m_command.execute();
  }

  @Override
  public void end(boolean interrupted) {
    m_command.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return m_drive.getPose().getTranslation().getDistance(m_targetPose.get().getTranslation()) < 0.2
        && m_drive.getPose().getRotation().minus(m_targetPose.get().getRotation()).getDegrees()
            < 5.0;
  }
}
