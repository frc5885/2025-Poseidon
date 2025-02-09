package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class QuestNavIOReal implements QuestNavIO {
  // NetworkTables instance and table for Quest communication
  private final NetworkTableInstance m_nt4Instance = NetworkTableInstance.getDefault();
  private final NetworkTable m_nt4Table = m_nt4Instance.getTable("questnav");

  // NetworkTables subscribers/publishers for Quest status
  private final IntegerSubscriber m_questMiso = m_nt4Table.getIntegerTopic("miso").subscribe(0);
  private final IntegerPublisher m_questMosi = m_nt4Table.getIntegerTopic("mosi").publish();
  private final DoubleSubscriber m_questTimestamp =
      m_nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private final FloatArraySubscriber m_questPosition =
      m_nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private final FloatArraySubscriber m_questQuaternion =
      m_nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
  private final FloatArraySubscriber m_questEulerAngles =
      m_nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private final DoubleSubscriber m_questBatteryPercent =
      m_nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    inputs.connected =
        ((Timer.getTimestamp() - m_questBatteryPercent.getLastChange()) / 1000) < 250;
    inputs.timestamp = m_questTimestamp.get();
    inputs.batteryPercent = m_questBatteryPercent.get();
    inputs.position = m_questPosition.get();
    inputs.quaternion = m_questQuaternion.get();
    inputs.eulerAngles = m_questEulerAngles.get();
  }

  @Override
  public void cleanUpQuestNavMessages() {
    if (m_questMiso.get() == 99) {
      m_questMosi.set(0);
    }
  }
}
