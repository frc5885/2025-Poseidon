// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class QuestNav extends SubsystemBase {
  // CONSTANTS
  private Translation2d robotToQuestTransform = new Translation2d(0, Units.inchesToMeters(12));
  private Translation2d questToFieldTransform = new Translation2d();

  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questQuaternion =
      nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questEulerAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent =
      nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  // Local heading helper variables
  private float yaw_offset = 0.0f;
  private Pose2d resetPosition = new Pose2d();

  public QuestNav() {
    resetPose();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    cleanUpQuestNavMessages();
    Logger.recordOutput("Odometry/Quest", getPose());
  }

  public void resetPose() {
    zeroHeading();
    zeroPosition();
    questToFieldTransform = getQuestNavPose().minus(resetPosition).getTranslation();
  }

  // Gets the Quest's measured position.
  public Pose2d getPose() {
    return new Pose2d(
        getQuestNavPose().minus(resetPosition).getTranslation().minus(questToFieldTransform),
        Rotation2d.fromDegrees(-getOculusYaw()));
  }

  // Gets the battery percent of the Quest.
  public double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  // Returns if the Quest is connected.
  public boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Gets the Quaternion of the Quest.
  public Quaternion getQuaternion() {
    float[] qqFloats = questQuaternion.get();
    return new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]);
  }

  // Gets the Quests's timestamp.
  public double timestamp() {
    return questTimestamp.get();
  }

  // Zero the relative robot heading
  public void zeroHeading() {
    float[] eulerAngles = questEulerAngles.get();
    yaw_offset = eulerAngles[1];
  }

  // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
  public void zeroPosition() {
    resetPosition = getPose();
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  // Clean up questnav subroutine messages after processing on the headset
  public void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  // Get the yaw Euler angle of the headset
  private float getOculusYaw() {
    float[] eulerAngles = questEulerAngles.get();
    var ret = eulerAngles[1] - yaw_offset;
    ret %= 360;
    if (ret < 0) {
      ret += 360;
    }
    return ret;
  }

  private Translation2d getQuestNavTranslation() {
    float[] questnavPosition = questPosition.get();
    return new Translation2d(questnavPosition[2], -questnavPosition[0]);
  }

  private Pose2d getQuestNavPose() {
    var oculousPositionCompensated = getQuestNavTranslation().minus(robotToQuestTransform);
    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getOculusYaw()));
  }
}
