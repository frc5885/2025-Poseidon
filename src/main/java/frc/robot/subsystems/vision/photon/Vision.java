// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision.photon;

import static frc.robot.subsystems.vision.photon.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.vision.photon.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.photon.VisionIO.PoseObservationType;
import frc.robot.util.AllianceFlipUtil;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.IntStream;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer m_consumer;
  private final VisionIO[] m_io;
  private final VisionIOInputsAutoLogged[] m_inputs;
  private final Alert[] m_disconnectedAlerts;
  // private final SparkMax m_power = new SparkMax(VisionConstants.kCameraPowerId,
  // MotorType.kBrushed);
  // private final PWM m_source = new PWM(kCameraPowerChannel);

  // if -1, use  measurements from all tags
  // if >= 0, use measurements from only the optimal tag and camera for the specified post ID
  @Setter private static int singleTargetPostID = -1;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    m_consumer = consumer;
    m_io = io;
    // m_source.setPulseTimeMicroseconds(1550);

    // Initialize inputs
    m_inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < m_inputs.length; i++) {
      m_inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    m_disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < m_inputs.length; i++) {
      m_disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple aiming with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return m_inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < m_io.length; i++) {
      m_io[i].updateInputs(m_inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), m_inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < m_io.length; cameraIndex++) {
      // Update disconnected alert
      m_disconnectedAlerts[cameraIndex].set(!m_inputs[cameraIndex].connected);

      if (m_io[cameraIndex].getCameraType() == VisionIO.CameraType.APRILTAG) {
        // Initialize logging values
        List<Pose3d> tagPoses = new LinkedList<>();
        List<Pose3d> robotPoses = new LinkedList<>();
        List<Pose3d> robotPosesAccepted = new LinkedList<>();
        List<Pose3d> robotPosesRejected = new LinkedList<>();

        // Add tag poses
        for (int tagId : m_inputs[cameraIndex].tagIds) {
          var tagPose = kAprilTagLayout.getTagPose(tagId);
          if (tagPose.isPresent()) {
            tagPoses.add(tagPose.get());
          }
        }

        // Loop over pose observations
        for (PoseObservation observation : m_inputs[cameraIndex].poseObservations) {
          // Check whether to reject pose
          boolean rejectPose =
              observation.tagCount() == 0 // Must have at least one tag
                  || (observation.tagCount() == 1
                      && observation.ambiguity() > kMaxAmbiguity) // Cannot be high ambiguity
                  || Math.abs(observation.pose().getZ())
                      > kMaxZError // Must have realistic Z coordinate

                  // Must be within the field boundaries
                  || observation.pose().getX() < 0.0
                  || observation.pose().getX() > kAprilTagLayout.getFieldLength()
                  || observation.pose().getY() < 0.0
                  || observation.pose().getY() > kAprilTagLayout.getFieldWidth();

          // any pose with tags not in lowTags is BANNED
          // this line is crazy elegant and straight from claude
          // if (Arrays.stream(m_inputs[cameraIndex].tagIds).anyMatch(id -> !lowTags.contains(id)))
          // {
          //   rejectPose = true;
          // }

          // If singleTargetPostID is set, only use measurements from the optimal tag and camera
          if (singleTargetPostID >= 0) {
            boolean isFlipped = AllianceFlipUtil.shouldFlip();
            int trustedTag = getTrustedTag(singleTargetPostID, isFlipped);
            int trustedCamera = getTrustedCamera(singleTargetPostID);
            Logger.recordOutput("Vision/SingleTargetMode/TrustedTag", trustedTag);
            Logger.recordOutput(
                "Vision/SingleTargetMode/TrustedCamera",
                trustedCamera == 0 ? kCamera0Name : kCamera1Name);
            // reject if wrong camera
            if (cameraIndex != trustedCamera) {
              rejectPose = true;
              // reject if wrong tag
            } else if (Arrays.stream(m_inputs[cameraIndex].tagIds)
                .noneMatch(id -> id == trustedTag)) {
              rejectPose = true;
            }
          }
          Logger.recordOutput("Vision/SingleTargetMode/Enabled", singleTargetPostID >= 0);

          // Add pose to log
          robotPoses.add(observation.pose());
          if (rejectPose) {
            robotPosesRejected.add(observation.pose());
          } else {
            robotPosesAccepted.add(observation.pose());
          }

          // Skip if rejected
          if (rejectPose) {
            continue;
          }

          // Calculate standard deviations
          double stdDevFactor =
              Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();

          double linearStdDev = kLinearStdDevBaseline * stdDevFactor;
          double angularStdDev = kAngularStdDevBaseline * stdDevFactor;
          if (observation.type() == PoseObservationType.MEGATAG_2) {
            linearStdDev *= kLinearStdDevMegatag2Factor;
            angularStdDev *= kAngularStdDevMegatag2Factor;
          }
          if (cameraIndex < kCameraStdDevFactors.length) {
            linearStdDev *= kCameraStdDevFactors[cameraIndex];
            angularStdDev *= kCameraStdDevFactors[cameraIndex];
          }

          // Send vision observation
          m_consumer.accept(
              observation.pose().toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev),
              observation.averageTagDistance());
        }

        // Log camera datadata
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
            tagPoses.toArray(new Pose3d[tagPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
            robotPoses.toArray(new Pose3d[robotPoses.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
            robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
            robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
        allTagPoses.addAll(tagPoses);
        allRobotPoses.addAll(robotPoses);
        allRobotPosesAccepted.addAll(robotPosesAccepted);
        allRobotPosesRejected.addAll(robotPosesRejected);

        // Coral camera
      } else if (m_io[cameraIndex].getCameraType() == VisionIO.CameraType.CORAL) {
        Logger.recordOutput(
            "Vision/Camera" + Integer.toString(cameraIndex) + "/LatestTarget",
            new Translation2d(
                m_inputs[cameraIndex].latestTargetObservation.tx().getRadians(),
                m_inputs[cameraIndex].latestTargetObservation.ty().getRadians()));
      }
    }

    boolean anyDisconnected = IntStream.range(0, m_io.length).anyMatch(i -> !m_inputs[i].connected);
    LEDSubsystem.getInstance().setPhotonDied(anyDisconnected);

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs,
        double averageTagDistance);
  }
}
