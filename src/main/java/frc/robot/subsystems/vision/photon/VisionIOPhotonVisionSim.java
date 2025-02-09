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
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim m_visionSim;

  private final Supplier<Pose2d> m_poseSupplier;
  private final PhotonCameraSim m_cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);
    m_poseSupplier = poseSupplier;

    // Initialize vision sim
    if (m_visionSim == null) {
      m_visionSim = new VisionSystemSim("main");
      m_visionSim.addAprilTags(kAprilTagLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    m_cameraSim = new PhotonCameraSim(m_camera, cameraProperties);
    m_visionSim.addCamera(m_cameraSim, robotToCamera);

    // Enable camera streams in simulation
    boolean renderSim = true;
    m_cameraSim.enableRawStream(renderSim);
    m_cameraSim.enableProcessedStream(renderSim);
    m_cameraSim.enableDrawWireframe(renderSim);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    m_visionSim.update(m_poseSupplier.get());
    super.updateInputs(inputs);
  }
}
