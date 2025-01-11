package frc.robot.subsystems.drive.QuestNav;

import org.littletonrobotics.junction.AutoLog;

public interface QuestNavIO {
  @AutoLog
  public static class QuestNavIOInputs {
    public boolean connected = false;
    public double timestamp = 0.0;
    public double batteryPercent = 0.0;
    public float[] position = new float[] {0.0f, 0.0f, 0.0f};
    public float[] quaternion = new float[] {0.0f, 0.0f, 0.0f, 0.0f};
    public float[] eulerAngles = new float[] {0.0f, 0.0f, 0.0f};
  }

  public default void updateInputs(QuestNavIOInputs inputs) {}

  /** Clean up questnav subroutine messages after they've been processed on the headset. */
  public default void cleanUpQuestNavMessages() {}
}
