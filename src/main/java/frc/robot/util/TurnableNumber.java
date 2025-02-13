package frc.robot.util;

import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TurnableNumber implements DoubleSupplier {
  private double m_initalValue;
  private String m_key;
  private LoggedNetworkNumber m_dashboardNumber;
  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   */
  public TurnableNumber(String key) {
    m_initalValue = 0.0;
    m_key = "/turning" + "/" + key;

    if (Constants.TurningMode) {
      m_dashboardNumber = new LoggedNetworkNumber(m_key, 0.0);
    }
  }
  /**
   * Create a new LoggedTunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TurnableNumber(String key, double initalValue) {
    m_initalValue = initalValue;
    m_key = "/turning" + "/" + key;
    if (Constants.TurningMode) {
      m_dashboardNumber = new LoggedNetworkNumber(m_key, m_initalValue);
    }
  }

  @Override
  public double getAsDouble() {
    return Constants.TurningMode ? m_dashboardNumber.get() : m_initalValue;
  }

  public boolean hasChanged() {
    if (m_dashboardNumber.get() != m_initalValue) {
      return true;
    }
    return false;
  }

  /** print your key and turned value to the console */
  public void turnedReminder() {
    if (hasChanged()) {
      System.out.println(m_key + ": " + m_dashboardNumber.get());
    } else {
      System.out.println(m_key + ": " + m_initalValue);
    }
  }
}
