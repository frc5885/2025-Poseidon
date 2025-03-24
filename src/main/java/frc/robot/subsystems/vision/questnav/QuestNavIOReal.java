package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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

  /** Subscriber for heartbeat requests */
  private final DoubleSubscriber heartbeatRequestSub =
      m_nt4Table.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);
  /** Publisher for heartbeat responses */
  private final DoublePublisher heartbeatResponsePub =
      m_nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish();
  /** Last processed heartbeat request ID */
  private double lastProcessedHeartbeatId = 0;

  private double m_prevTimestamp = 0.0;
  private Debouncer m_connectedDebouncer = new Debouncer(0.5);

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    inputs.timestamp = m_questTimestamp.get();
    inputs.connected = !m_connectedDebouncer.calculate(inputs.timestamp == m_prevTimestamp);
    m_prevTimestamp = inputs.timestamp;
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

  @Override
  /** Process heartbeat requests from Quest and respond with the same ID */
  public void processHeartbeat() {
    double requestId = heartbeatRequestSub.get();
    // Only respond to new requests to avoid flooding
    if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
      heartbeatResponsePub.set(requestId);
      lastProcessedHeartbeatId = requestId;
    }
  }
}
