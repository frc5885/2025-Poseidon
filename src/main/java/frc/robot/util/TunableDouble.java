package frc.robot.util;

import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableDouble implements DoubleSupplier {
  private final String m_key;
  private boolean hasDefault = false;
  private double m_defaultValue;
  private LoggedNetworkNumber m_entry;

  private TunableDouble(String key) {
    m_key = "/Tuning/" + key;
  }

  private TunableDouble(String key, double defaultValue) {
    this(key);
    initDefault(defaultValue);
  }

  public static TunableDouble register(String key) {
    return new TunableDouble(key);
  }

  public static TunableDouble register(String key, double defaultValue) {
    return new TunableDouble(key, defaultValue);
  }

  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      m_defaultValue = defaultValue;
      if (Constants.kTuning) {
        m_entry = new LoggedNetworkNumber(m_key, m_defaultValue);
      }
    }
  }

  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else {
      return Constants.kTuning ? m_entry.get() : m_defaultValue;
    }
  }

  @Override
  public double getAsDouble() {
    return get();
  }
}
