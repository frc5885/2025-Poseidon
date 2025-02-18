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
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.GamePieces.CoralTargetModel;
import java.util.function.Supplier;
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
        // Add coral targets
        TargetModel coralModel = CoralTargetModel.getCoralModel();
        m_visionSimCoral.addVisionTargets(
            "coral",
            CoralTargetModel.getCoralPositions().stream()
                .map(pos -> new VisionTargetSim(pos, coralModel))
                .toArray(VisionTargetSim[]::new));
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
    boolean renderSim = false;
    m_cameraSim.enableRawStream(renderSim);
    m_cameraSim.enableProcessedStream(renderSim);
    m_cameraSim.enableDrawWireframe(renderSim);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    m_visionSimAprilTags.update(m_poseSupplier.get());
    m_visionSimCoral.update(m_poseSupplier.get());
    super.updateInputs(inputs);
  }
}
