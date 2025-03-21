// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

public class ChassisTrapezoidalController {
  private TrapezoidProfile m_xProfile;
  private TrapezoidProfile m_yProfile;
  private TrapezoidProfile m_thetaProfile;

  private TrapezoidProfile.State m_xGoalState;
  private TrapezoidProfile.State m_yGoalState;
  private TrapezoidProfile.State m_thetaGoalState;

  private TrapezoidProfile.State m_xPrevSetpoint;
  private TrapezoidProfile.State m_yPrevSetpoint;
  private TrapezoidProfile.State m_thetaPrevSetpoint;

  private PIDController m_xController;
  private PIDController m_yController;
  private PIDController m_thetaController;

  public ChassisTrapezoidalController(
      TrapezoidProfile.Constraints xyConstraints,
      TrapezoidProfile.Constraints thetaConstraints,
      PIDController xController,
      PIDController yController,
      PIDController thetaController) {
    m_xProfile = new TrapezoidProfile(xyConstraints);
    m_yProfile = new TrapezoidProfile(xyConstraints);
    m_thetaProfile = new TrapezoidProfile(thetaConstraints);

    m_xGoalState = new TrapezoidProfile.State(0, 0);
    m_yGoalState = new TrapezoidProfile.State(0, 0);
    m_thetaGoalState = new TrapezoidProfile.State(0, 0);

    m_xPrevSetpoint = m_xGoalState;
    m_yPrevSetpoint = m_yGoalState;
    m_thetaPrevSetpoint = m_thetaGoalState;

    m_xController = xController;
    m_yController = yController;
    m_thetaController = thetaController;
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setGoal(
      TrapezoidProfile.State xGoal,
      TrapezoidProfile.State yGoal,
      TrapezoidProfile.State thetaGoal) {
    m_xGoalState = xGoal;
    m_yGoalState = yGoal;
    m_thetaGoalState = thetaGoal;

    Logger.recordOutput(
        "Odometry/ChassisController/Goal",
        new Pose2d(xGoal.position, yGoal.position, Rotation2d.fromRadians(thetaGoal.position)));
  }

  public void setGoalPose(Pose2d goal) {
    setGoal(
        new TrapezoidProfile.State(goal.getX(), 0),
        new TrapezoidProfile.State(goal.getY(), 0),
        new TrapezoidProfile.State(goal.getRotation().getRadians(), 0));
  }

  public void setCurrentState(Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, robotPose.getRotation());
    m_xPrevSetpoint =
        new TrapezoidProfile.State(robotPose.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
    m_yPrevSetpoint =
        new TrapezoidProfile.State(robotPose.getY(), fieldRelativeSpeeds.vyMetersPerSecond);
    m_thetaPrevSetpoint =
        new TrapezoidProfile.State(
            robotPose.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond);

    m_xController.reset();
    m_yController.reset();
    m_thetaController.reset();
  }

  public ChassisSpeeds calculate(Pose2d robotPose) {
    double dt = 0.02;
    TrapezoidProfile.State x = m_xProfile.calculate(dt, m_xPrevSetpoint, m_xGoalState);
    TrapezoidProfile.State y = m_yProfile.calculate(dt, m_yPrevSetpoint, m_yGoalState);
    TrapezoidProfile.State theta =
        m_thetaProfile.calculate(dt, m_thetaPrevSetpoint, m_thetaGoalState);
    m_xPrevSetpoint = x;
    m_yPrevSetpoint = y;
    m_thetaPrevSetpoint = theta;

    Logger.recordOutput(
        "Odometry/ChassisController/Setpoint",
        new Pose2d(x.position, y.position, Rotation2d.fromRadians(theta.position)));

    double xOutput = m_xController.calculate(robotPose.getX(), x.position);
    double yOutput = m_yController.calculate(robotPose.getY(), y.position);
    double thetaOutput =
        m_thetaController.calculate(robotPose.getRotation().getRadians(), theta.position);

    return new ChassisSpeeds(xOutput, yOutput, thetaOutput);
  }

  public boolean isGoalAchieved() {
    return m_xController.atSetpoint()
        && m_xController.getSetpoint() == m_xGoalState.position
        && m_yController.atSetpoint()
        && m_yController.getSetpoint() == m_yGoalState.position
        && m_thetaController.atSetpoint()
        && m_thetaController.getSetpoint() == m_thetaGoalState.position;
  }
}
