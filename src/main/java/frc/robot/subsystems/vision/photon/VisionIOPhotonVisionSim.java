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

import static frc.robot.subsystems.vision.photon.VisionConstants.kAprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.GamePieces.CoralTargetModel;
import java.util.Arrays;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  // Static means they're global across cameras
  private static VisionSystemSim m_visionSimAprilTags;
  private static VisionSystemSim m_visionSimCoral;

  // these are so that we only call update once per cycle even if we have multiple cameras
  private static boolean m_mainAprilTagCameraSet = false;
  private static boolean m_mainCoralCameraSet = false;
  private final boolean m_isMainCamera;

  private static final TargetModel coralModel = CoralTargetModel.getCoralModel();

  private final Supplier<Pose2d> m_poseSupplier;
  private final PhotonCameraSim m_cameraSim;

  private final CameraType m_cameraType;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d robotToCamera,
      Supplier<Pose2d> poseSupplier,
      CameraType cameraType) {
    super(name, robotToCamera, cameraType);
    m_poseSupplier = poseSupplier;
    m_cameraType = cameraType;

    // Check if this is the main camera
    if (m_cameraType == CameraType.APRILTAG && !m_mainAprilTagCameraSet) {
      m_mainAprilTagCameraSet = true;
      m_isMainCamera = true;
    } else if (m_cameraType == CameraType.CORAL && !m_mainCoralCameraSet) {
      m_mainCoralCameraSet = true;
      m_isMainCamera = true;
    } else {
      m_isMainCamera = false;
    }

    // Initialize vision sim
    if (m_cameraType == CameraType.APRILTAG) {
      if (m_visionSimAprilTags == null) {
        m_visionSimAprilTags = new VisionSystemSim("april_tags");
        // Load april tag layout
        m_visionSimAprilTags.addAprilTags(kAprilTagLayout);
      }
    } else if (m_cameraType == CameraType.CORAL) {
      if (m_visionSimCoral == null) {
        m_visionSimCoral = new VisionSystemSim("coral");
        // Initialize with empty targets - will be updated dynamically
        m_visionSimCoral.addVisionTargets("coral");
      }
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties);
    if (m_cameraType == CameraType.APRILTAG) {
      m_visionSimAprilTags.addCamera(m_cameraSim, robotToCamera);
    } else if (m_cameraType == CameraType.CORAL) {
      m_visionSimCoral.addCamera(m_cameraSim, robotToCamera);
    }

    // Enable camera streams in simulation
    boolean renderSim = true;
    m_cameraSim.enableRawStream(renderSim);
    m_cameraSim.enableProcessedStream(renderSim);
    m_cameraSim.enableDrawWireframe(renderSim);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    if (m_cameraType == CameraType.APRILTAG && m_isMainCamera) {
      m_visionSimAprilTags.update(m_poseSupplier.get());
    } else if (m_cameraType == CameraType.CORAL && m_isMainCamera) {
      // Get coral positions from simulated arena
      Pose3d[] coralPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Coral");

      // Update vision targets with new coral positions
      m_visionSimCoral.removeVisionTargets("coral");
      if (coralPoses != null && coralPoses.length > 0) {
        VisionTargetSim[] targets =
            Arrays.stream(coralPoses)
                .map(pose -> new VisionTargetSim(pose, coralModel))
                .toArray(VisionTargetSim[]::new);
        m_visionSimCoral.addVisionTargets("coral", targets);
      }

      m_visionSimCoral.update(m_poseSupplier.get());
    }
    super.updateInputs(inputs);
  }
}
