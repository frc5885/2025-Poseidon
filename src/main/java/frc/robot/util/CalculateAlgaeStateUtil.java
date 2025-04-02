// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import java.util.ArrayList;

/** Add your docs here. */
public class CalculateAlgaeStateUtil {
  public static SuperStructureState calculateIntakeState(Pose2d pose) {
    ArrayList<Pose2d> reefFaces = FieldConstants.getReefFaces();

    for (int i = 0; i < reefFaces.size(); i++) {
      if (reefFaces.get(i).equals(pose)) {
        return FieldConstants.Reef.AlgaeLevel[i];
      }
    }

    return SuperStructureState.INTAKE_ALGAE_L2; // Default value
  }

  public static SuperStructureState calculateAfterIntakeState(SuperStructureState currentState) {
    return currentState == SuperStructureState.INTAKE_ALGAE_L2
        ? SuperStructureState.AFTER_ALGAE_L2
        : SuperStructureState.AFTER_ALGAE_L3;
  }
}
