// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import java.util.stream.IntStream;

/** Add your docs here. */
public class CalculateAlgaeStateUtil {
  public static SuperStructureState calculateIntakeState(Pose2d pose) {
    return IntStream.range(0, FieldConstants.Reef.centerFaces.length)
        .filter(i -> FieldConstants.Reef.centerFaces[i].equals(AllianceFlipUtil.apply(pose)))
        .mapToObj(i -> FieldConstants.Reef.AlgaeLevel[i])
        .findFirst()
        .orElse(SuperStructureState.INTAKE_ALGAE_L2);
  }

  public static SuperStructureState calculateAfterIntakeState(SuperStructureState currentState) {

    return currentState == SuperStructureState.INTAKE_ALGAE_L2
        ? SuperStructureState.AFTER_ALGAE_L2
        : SuperStructureState.AFTER_ALGAE_L3;
  }
}
