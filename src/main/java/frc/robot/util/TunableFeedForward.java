// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A PID controller that can be tuned on SmartDashboard. */
public class TunableFeedForward extends ArmFeedforward implements Sendable {

  private final String m_key;
  private final boolean m_tuneMode;

  public TunableFeedForward(
      double ks, double kg, double kv, double ka, String key, boolean tuneMode) {
    super(ks, kg, kv, ka, 0.020);
    m_key = key + "/FeedForward";
    m_tuneMode = tuneMode;

    if (m_tuneMode) {
      SmartDashboard.putData(m_key, this);
      //   SmartDashboard.putNumber(m_key + "/S", getKs());
      //   SmartDashboard.putNumber(m_key + "/G", getKg());
      //   SmartDashboard.putNumber(m_key + "/V", getKv());
      //   SmartDashboard.putNumber(m_key + "/A" getKa());

    }
  }

  @Override
  public double calculate(double positionRadians, double velocity) {
    if (m_tuneMode) {
      double oldS = getKs();
      double oldG = getKg();
      double oldV = getKv();
      double oldA = getKa();

      double newS = SmartDashboard.getNumber(m_key + "/S", oldS);
      double newG = SmartDashboard.getNumber(m_key + "/G", oldG);
      double newV = SmartDashboard.getNumber(m_key + "/V", oldV);
      double newA = SmartDashboard.getNumber(m_key + "/A", oldA);

      if (newS != oldS) {
        setKs(newS);
      }
      if (newG != oldG) {
        setKg(newG);
      }
      if (newV != oldV) {
        setKv(newV);
      }
      if (newA != oldA) {
        setKa(newA);
      }
    }
    return super.calculate(positionRadians, velocity, 0.0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("FeedForward");
    builder.addDoubleProperty("s", this::getKs, this::setKs);
    builder.addDoubleProperty("g", this::getKg, this::setKg);
    builder.addDoubleProperty("v", this::getKv, this::setKv);
    builder.addDoubleProperty("a", this::getKa, this::setKa);
  }
}
