// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A PID controller that can be tuned on SmartDashboard. */
public class TunablePIDController extends PIDController {

  private final String m_key;
  private final boolean m_tuneMode;

  public TunablePIDController(
      double kP, double kI, double kD, double toleranceRad, String key, boolean tuneMode) {
    super(kP, kI, kD);
    setTolerance(toleranceRad);
    m_key = key + "/PID";
    m_tuneMode = tuneMode;

    if (m_tuneMode) {
      SmartDashboard.putData(m_key, this);
    }
  }

  @Override
  public double calculate(double measurement) {
    if (m_tuneMode) {
      double oldP = getP();
      double oldI = getI();
      double oldD = getD();

      double newP = SmartDashboard.getNumber(m_key + "/p", oldP);
      double newI = SmartDashboard.getNumber(m_key + "/i", oldI);
      double newD = SmartDashboard.getNumber(m_key + "/d", oldD);

      if (newP != oldP) {
        setP(newP);
      }
      if (newI != oldI) {
        setI(newI);
      }
      if (newD != oldD) {
        setD(newD);
      }
    }
    return super.calculate(measurement);
  }
}
