// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

public class ChassisTrapezoidalController {
  private TrapezoidProfile m_translateProfile;
  private TrapezoidProfile m_thetaProfile;

  private Pose2d m_goalPose;
  private Pose2d m_finalGoalPose;
  private Pose2d m_currentPose;

  private TrapezoidProfile.State m_translateGoalState;
  private TrapezoidProfile.State m_thetaGoalState;

  private TrapezoidProfile.State m_translatePrevSetpoint;
  private TrapezoidProfile.State m_thetaPrevSetpoint;

  private PIDController m_translateController;
  private PIDController m_thetaController;

  private Debouncer m_atGoalDwellDebouncer = new Debouncer(0.1); // hehehehehe

  public ChassisTrapezoidalController(
      TrapezoidProfile.Constraints translateConstraints,
      TrapezoidProfile.Constraints thetaConstraints,
      PIDController translateController,
      PIDController thetaController) {
    m_translateProfile = new TrapezoidProfile(translateConstraints);
    m_thetaProfile = new TrapezoidProfile(thetaConstraints);

    m_goalPose = new Pose2d();
    m_finalGoalPose = new Pose2d();
    m_currentPose = new Pose2d();

    m_translateGoalState = new TrapezoidProfile.State(0, 0);
    m_thetaGoalState = new TrapezoidProfile.State(0, 0);

    m_translatePrevSetpoint = m_translateGoalState;
    m_thetaPrevSetpoint = m_thetaGoalState;

    m_translateController = translateController;
    m_thetaController = thetaController;
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void setGoal(TrapezoidProfile.State translateGoal, TrapezoidProfile.State thetaGoal) {
    m_translateGoalState = translateGoal;
    m_thetaGoalState = thetaGoal;

    Logger.recordOutput(
        "Odometry/ChassisController/Goal",
        new Pose2d(
            m_goalPose.getX(), m_goalPose.getY(), Rotation2d.fromRadians(thetaGoal.position)));
  }

  public void setGoalPose(Pose2d goal) {
    m_goalPose = goal;
    setGoal(
        new TrapezoidProfile.State(0, 0), // Target is 0 distance remaining
        new TrapezoidProfile.State(goal.getRotation().getRadians(), 0));
  }

  public void setFinalGoalPose(Pose2d goal) {
    m_finalGoalPose = goal;
  }

  public Pose2d getGoalPose() {
    return m_goalPose;
  }

  public Pose2d getFinalGoalPose() {
    return m_finalGoalPose;
  }

  public void setCurrentState(Pose2d robotPose, ChassisSpeeds chassisSpeeds) {
    m_currentPose = robotPose;

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, robotPose.getRotation());

    // For translate, we track current distance to goal
    double distanceToGoal = robotPose.getTranslation().getDistance(m_goalPose.getTranslation());

    // Calculate current speed in direction of goal
    Translation2d directionToGoal = robotPose.getTranslation().minus(m_goalPose.getTranslation());
    double currentTranslateSpeed = 0.0;
    if (directionToGoal.getNorm() > 1e-6) {
      directionToGoal = directionToGoal.div(directionToGoal.getNorm());
      currentTranslateSpeed =
          fieldRelativeSpeeds.vxMetersPerSecond * directionToGoal.getX()
              + fieldRelativeSpeeds.vyMetersPerSecond * directionToGoal.getY();
    }

    m_translatePrevSetpoint = new TrapezoidProfile.State(distanceToGoal, currentTranslateSpeed);
    m_thetaPrevSetpoint =
        new TrapezoidProfile.State(
            robotPose.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond);

    m_translateController.reset();
    m_thetaController.reset();
  }

  public ChassisSpeeds calculate(Pose2d robotPose) {
    double dt = 0.02;
    m_currentPose = robotPose;

    // Calculate distance error to goal
    double distanceToGoal = robotPose.getTranslation().getDistance(m_goalPose.getTranslation());
    Logger.recordOutput("Odometry/ChassisController/DistanceError", distanceToGoal);

    // Generate profile for translation (goal is to reach 0 distance)
    TrapezoidProfile.State translateSetpoint =
        m_translateProfile.calculate(dt, m_translatePrevSetpoint, m_translateGoalState);

    // Calculate direction vector to goal
    Translation2d errorVector = m_goalPose.getTranslation().minus(robotPose.getTranslation());
    Translation2d directionVector;

    if (errorVector.getNorm() > 1e-6) {
      directionVector = errorVector.div(errorVector.getNorm());
    } else {
      directionVector = new Translation2d(0, 0);
    }

    // Calculate translate velocity using PID
    double translateVelocity =
        -m_translateController.calculate(distanceToGoal, translateSetpoint.position);

    // Calculate x and y components of velocity
    double vxMetersPerSecond = directionVector.getX() * translateVelocity;
    double vyMetersPerSecond = directionVector.getY() * translateVelocity;

    // Generate profile for rotation
    // Get error which is the smallest distance between goal and measurement
    double goalMinDistance =
        MathUtil.angleModulus(m_thetaGoalState.position - robotPose.getRotation().getRadians());
    double setpointMinDistance =
        MathUtil.angleModulus(m_thetaPrevSetpoint.position - robotPose.getRotation().getRadians());
    Logger.recordOutput("Odometry/ChassisController/ThetaError", goalMinDistance);

    // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
    // may be outside the input range after this operation, but that's OK because the controller
    // will still go there and report an error of zero. In other words, the setpoint only needs to
    // be offset from the measurement by the input range modulus; they don't need to be equal.
    m_thetaGoalState.position = goalMinDistance + robotPose.getRotation().getRadians();
    m_thetaPrevSetpoint.position = setpointMinDistance + robotPose.getRotation().getRadians();
    TrapezoidProfile.State thetaSetpoint =
        m_thetaProfile.calculate(dt, m_thetaPrevSetpoint, m_thetaGoalState);

    // Calculate angular velocity using PID
    double omegaRadiansPerSecond =
        m_thetaController.calculate(robotPose.getRotation().getRadians(), thetaSetpoint.position);

    // Update previous setpoints
    m_translatePrevSetpoint = translateSetpoint;
    m_thetaPrevSetpoint = thetaSetpoint;

    // Log setpoint
    Translation2d setpointTranslation =
        robotPose
            .getTranslation()
            .plus(directionVector.times(distanceToGoal - translateSetpoint.position));

    Logger.recordOutput(
        "Odometry/ChassisController/Setpoint",
        new Pose2d(
            setpointTranslation.getX(),
            setpointTranslation.getY(),
            Rotation2d.fromRadians(thetaSetpoint.position)));

    Logger.recordOutput(
        "Odometry/ChassisController/GoalHypotenuse",
        Math.hypot(m_finalGoalPose.getX(), m_finalGoalPose.getY()));
    Logger.recordOutput(
        "Odometry/ChassisController/RobotHypotenuse",
        Math.hypot(robotPose.getX(), robotPose.getY()));

    // Return robot-relative speeds
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, robotPose.getRotation());
  }

  public boolean isGoalAchieved() {
    double distanceThreshold = m_translateController.getErrorTolerance();
    double angleThreshold = m_thetaController.getErrorTolerance();

    Logger.recordOutput(
        "Odometry/ChassisController/AngleAtSetpoint",
        Math.abs(
                MathUtil.angleModulus(
                    m_currentPose.getRotation().getRadians()
                        - m_goalPose.getRotation().getRadians()))
            < angleThreshold);
    Logger.recordOutput(
        "Odometry/ChassisController/TranslateAtSetpoint",
        m_currentPose.getTranslation().getDistance(m_finalGoalPose.getTranslation())
            < distanceThreshold);

    boolean isAchieved =
        m_currentPose.getTranslation().getDistance(m_finalGoalPose.getTranslation())
                < distanceThreshold
            && Math.abs(
                    MathUtil.angleModulus(
                        m_currentPose.getRotation().getRadians()
                            - m_goalPose.getRotation().getRadians()))
                < angleThreshold;
    // wait for goal to be achieved for a certain amount of time before ending
    return m_atGoalDwellDebouncer.calculate(isAchieved);
  }

  // Add this method to ChassisTrapezoidalController.java
  public void reset(Pose2d robotPose, ChassisSpeeds chassisSpeeds, Pose2d goalPose) {
    // Reset goal poses first
    m_goalPose = goalPose;
    m_finalGoalPose = goalPose;

    // Reset states
    m_translateGoalState = new TrapezoidProfile.State(0, 0);
    m_thetaGoalState = new TrapezoidProfile.State(goalPose.getRotation().getRadians(), 0);

    // Set current state after goals are properly initialized
    m_currentPose = robotPose;

    // Calculate distance and direction with the NEW goal
    double distanceToGoal = robotPose.getTranslation().getDistance(goalPose.getTranslation());

    // Initialize previous setpoints with current state
    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, robotPose.getRotation());

    // Calculate direction and speed toward the NEW goal
    Translation2d directionToGoal = robotPose.getTranslation().minus(goalPose.getTranslation());
    double currentTranslateSpeed = 0.0;
    if (directionToGoal.getNorm() > 1e-6) {
      directionToGoal = directionToGoal.div(directionToGoal.getNorm());
      currentTranslateSpeed =
          fieldRelativeSpeeds.vxMetersPerSecond * directionToGoal.getX()
              + fieldRelativeSpeeds.vyMetersPerSecond * directionToGoal.getY();
    }

    m_translatePrevSetpoint = new TrapezoidProfile.State(distanceToGoal, currentTranslateSpeed);
    m_thetaPrevSetpoint =
        new TrapezoidProfile.State(
            robotPose.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond);

    // Reset controllers
    m_translateController.reset();
    m_thetaController.reset();
  }
}
