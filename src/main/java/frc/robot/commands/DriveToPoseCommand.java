// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.Supplier;

public class DriveToPoseCommand extends Command {
  private final Drive m_drive;
  private Supplier<Pose2d> m_targetPose;
  private double m_distanceTolerance;
  private double m_rotationTolerance;
  private Command m_command;
  private Pose2d flippedPose;

  private boolean distanceTooShort = false;

  /** A command that drives the robot to a target pose using Pathplanner's AutoBuilder. */
  public DriveToPoseCommand(
      Drive drive,
      Supplier<Pose2d> targetPose,
      double distanceToleranceMeters,
      double rotationToleranceDegrees) {
    m_drive = drive;
    m_targetPose = targetPose;
    m_distanceTolerance = distanceToleranceMeters;
    m_rotationTolerance = rotationToleranceDegrees;

    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    flippedPose = AllianceFlipUtil.apply(m_targetPose.get());
    // Pathplanner won't generate a path if the distance is less than 0.6m
    if (m_drive.getPose().getTranslation().getDistance(flippedPose.getTranslation()) < 0.61) {
      distanceTooShort = true;
    } else {
      // need to add this or it never gets reset
      distanceTooShort = false;
    }
    m_command = m_drive.getDriveToPoseCommand(m_targetPose);
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

    return (m_drive.getPose().getTranslation().getDistance(flippedPose.getTranslation())
                < m_distanceTolerance
            && m_drive.getPose().getRotation().minus(flippedPose.getRotation()).getDegrees()
                < m_rotationTolerance)
        || distanceTooShort;
  }
}
