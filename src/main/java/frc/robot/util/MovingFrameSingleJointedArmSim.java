// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * A simulation of a single jointed arm that moves in a frame that is itself moving. This is useful
 * for simulating a wrist on a robot arm.
 */
public class MovingFrameSingleJointedArmSim extends SingleJointedArmSim {
  // The angle of the arm that the wrist is mounted on
  private double m_armAngle;

  private double m_armLengthMeters;
  private double m_minAngle;
  private double m_maxAngle;
  private boolean m_simulateGravity;

  public MovingFrameSingleJointedArmSim(
      DCMotor gearbox,
      double gearing,
      double jKgMetersSquared,
      double armLengthMeters,
      double minAngleRads,
      double maxAngleRads,
      boolean simulateGravity,
      double startingAngleRads,
      double startingArmAngleRads) {
    super(
        gearbox,
        gearing,
        jKgMetersSquared,
        armLengthMeters,
        minAngleRads,
        maxAngleRads,
        simulateGravity,
        startingAngleRads);

    m_armLengthMeters = armLengthMeters;
    m_minAngle = minAngleRads;
    m_maxAngle = maxAngleRads;
    m_simulateGravity = simulateGravity;
    m_armAngle = startingArmAngleRads;
  }

  /**
   * Sets the current angle of the arm that the wrist is attached to.
   *
   * @param armAngle The angle of the arm in radians.
   */
  public void setArmAngle(double armAngleRad) {
    m_armAngle = armAngleRad;
  }

  /** Overrides the default update method to account for the arm angle. */
  @Override
  protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
    Matrix<N2, N1> updatedXhat =
        NumericalIntegration.rkdp(
            (Matrix<N2, N1> x, Matrix<N1, N1> _u) -> {
              Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
              if (m_simulateGravity) {
                // Calculate the relative angle between the arm and the gravitational force
                // Use the absolute angle (arm angle + wrist's relative angle)
                double absoluteAngle = x.get(0, 0) + m_armAngle;
                double alphaGrav = 3.0 / 2.0 * -9.8 * Math.cos(absoluteAngle) / m_armLengthMeters;
                xdot = xdot.plus(VecBuilder.fill(0, alphaGrav));
              }
              return xdot;
            },
            currentXhat,
            u,
            dtSeconds);

    // We check for collision after updating xhat
    if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_minAngle, 0);
    }
    if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
      return VecBuilder.fill(m_maxAngle, 0);
    }
    return updatedXhat;
  }
}
