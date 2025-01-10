// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Subsystem to interface with a Meta Quest headset acting as a positional sensor. */
public class QuestNav extends SubsystemBase {

  private Pose2d realQuestPoseSim = new Pose2d();
  private Supplier<Pose2d> realRobotPoseSupplierSim;

  // Physical offset from the robot center to the Quest headset.
  // Quest is 16 inches in front of the robot center
  private final Transform2d robotToQuestTransform =
      new Transform2d(Units.inchesToMeters(16.0), 0, new Rotation2d());

  // This transform maps between the quest's local coordinate system and field coordinates.
  private Transform2d questToFieldTransform = new Transform2d();

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

  /** Creates a new QuestNav subsystem. Initializes the robot pose to a "blank" pose2d (0,0,0). */
  public QuestNav(Supplier<Pose2d> realRobotPoseSupplier) {
    setRobotPose(new Pose2d());
    realRobotPoseSupplierSim = realRobotPoseSupplier;
    realQuestPoseSim = new Pose2d().transformBy(robotToQuestTransform);
  }

  @Override
  public void periodic() {
    cleanUpQuestNavMessages();

    Logger.recordOutput("Odometry/Quest", getRobotPose());

    Pose2d robot = realRobotPoseSupplierSim.get();
    realQuestPoseSim =
        robot.transformBy(
            new Transform2d(robotToQuestTransform.getTranslation(), new Rotation2d()));
    Logger.recordOutput("Odometry/PhysicalQuestLocation", realQuestPoseSim);
  }

  /**
   * Sets the current robot pose in FIELD coordinates. Internally, this calculates and stores the
   * transform needed to map the Quest’s coordinate system to field coordinates.
   *
   * @param realRobotPose The desired Pose2d of the robot in field coordinates
   */
  public void setRobotPose(Pose2d realRobotPose) {
    Pose2d realQuestPose = realRobotPose.transformBy(robotToQuestTransform);
    questToFieldTransform = realQuestPose.minus(getRawQuestPose());
  }

  /**
   * Returns the current robot pose in FIELD coordinates, incorporating the Quest's local readings,
   * the physical offset of the Quest on the robot, and the transform set by setRobotPose().
   *
   * @return The Pose2d of the robot in field coordinates
   */
  public Pose2d getRobotPose() {
    return getRawQuestPose()
        .transformBy(questToFieldTransform)
        .transformBy(robotToQuestTransform.inverse());
  }

  /** Returns the Quest’s battery percentage. */
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  /** Indicates if the Quest device is connected (heuristic based on update timing). */
  public boolean connected() {
    // If the last battery update was within ~250ms, we consider it connected
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  /** Returns the raw Quaternion from the Quest. */
  public Quaternion getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
  }

  /** Returns the Quest’s timestamp (seconds from power on as a double). */
  public double timestamp() {
    return questTimestamp.get();
  }

  /** Clean up questnav subroutine messages after they've been processed on the headset. */
  private void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  /**
   * Returns the yaw angle from the Quest's eulerAngles, minus any internal offset. This is used
   * internally to compute the robot heading in local Quest coordinates.
   */
  private float getRawQuestYaw() {
    float[] eulerAngles = questEulerAngles.get();
    float ret = eulerAngles[1];
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return ret;
  }

  /** Returns the raw Quest HMD pose (X, Y, Z, rotation) */
  private Pose2d getRawQuestPose() {
    float[] questnavPosition = questPosition.get();

    Pose2d ret =
        new Pose2d(questnavPosition[2], -questnavPosition[0], new Rotation2d(getRawQuestYaw()));
    return realQuestPoseSim;
  }
}
