package frc.robot.subsystems.drive.QuestNav;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

public class QuestNavIOReal implements QuestNavIO {
  // NetworkTables instance and table for Quest communication
  private final NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  private final NetworkTable nt4Table = nt4Instance.getTable("questnav");

  // NetworkTables subscribers/publishers for Quest status
  private final IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private final IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();
  private final DoubleSubscriber questTimestamp =
      nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private final FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private final FloatArraySubscriber questQuaternion =
      nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
  private final FloatArraySubscriber questEulerAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private final DoubleSubscriber questBatteryPercent =
      nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    inputs.connected =
        ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
    inputs.timestamp = questTimestamp.get();
    inputs.batteryPercent = questBatteryPercent.get();
    inputs.position = questPosition.get();
    inputs.quaternion = questQuaternion.get();
    inputs.eulerAngles = questEulerAngles.get();
  }

  @Override
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }
}
