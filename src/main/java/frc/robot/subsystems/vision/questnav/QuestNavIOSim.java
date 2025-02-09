package frc.robot.subsystems.vision.questnav;

import static frc.robot.subsystems.vision.questnav.QuestNavConstants.kRobotToQuestTransform;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

public class QuestNavIOSim implements QuestNavIO {

  private final Supplier<Pose2d> robotPoseSupplier;

  public QuestNavIOSim(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
  }

  @Override
  public void updateInputs(QuestNavIOInputs inputs) {
    inputs.connected = true;
    inputs.timestamp = Timer.getFPGATimestamp();
    inputs.batteryPercent = 58.85;

    Pose2d robotPose = robotPoseSupplier.get();
    Pose2d questPose = robotPose.transformBy(kRobotToQuestTransform);
    inputs.position = new float[] {(float) -questPose.getY(), 0, (float) questPose.getX()};
    inputs.quaternion = new float[] {0, 0, 0, 1};
    inputs.eulerAngles = new float[] {0, (float) -questPose.getRotation().getDegrees(), 0};
  }
}
