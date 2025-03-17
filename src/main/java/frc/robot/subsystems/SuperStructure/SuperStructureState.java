package frc.robot.subsystems.SuperStructure;

import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public enum SuperStructureState {
  INTAKE_CORAL(ElevatorLevel.STOW, ArmGoals.INTAKE),
  INTAKE_ALGAE_FLOOR(ElevatorLevel.L2, ArmGoals.ALGAE_FLOOR),
  INTAKE_ALGAE_L2(ElevatorLevel.ALGAE_L2, ArmGoals.REEF),
  INTAKE_ALGAE_L3(ElevatorLevel.ALGAE_L3, ArmGoals.REEF),
  SCORE_CORAL_L1(ElevatorLevel.L1, ArmGoals.REEF),
  SCORE_CORAL_L2(ElevatorLevel.L2, ArmGoals.REEF),
  SCORE_CORAL_L3(ElevatorLevel.L3, ArmGoals.REEF),
  SCORE_CORAL_L4(ElevatorLevel.L4, ArmGoals.REEF),
  SCORE_ALGAE_PROCESSOR(ElevatorLevel.STOW, ArmGoals.INTAKE),
  SCORE_ALGAE_NET(ElevatorLevel.L4, ArmGoals.STOW),
  STOWED(ElevatorLevel.STOW, ArmGoals.STOW),
  IDLE(ElevatorLevel.STOW, ArmGoals.IDLE),
  // TRANSITION STATES
  // while the arm is swinging in/out of stowed state (to/from intake state), lock the wrist to
  // prevent it from crashing
  STOWING(ElevatorLevel.STOW, ArmGoals.STOW),
  UNSTOWING(ElevatorLevel.STOW, ArmGoals.INTAKE),
  // to prevent wrist from flopping
  IDLE_TO_INTAKE(ElevatorLevel.STOW, ArmGoals.INTAKE);

  public ElevatorLevel elevatorGoal;
  public ArmGoals armGoal;
}
