package frc.robot.subsystems.SuperStructure;

import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public enum SuperStructureState {
  STOWED(ElevatorLevel.STOW, ArmGoals.STOW),
  IDLE_CORAL(ElevatorLevel.IDLE, ArmGoals.IDLE_CORAL),
  IDLE_ALGAE(ElevatorLevel.STOW, ArmGoals.IDLE_ALGAE),
  INTAKE_CORAL(ElevatorLevel.STOW, ArmGoals.IDLE_CORAL),
  INTAKE_ALGAE_L2(ElevatorLevel.STOW, ArmGoals.ALGAE_L2),
  INTAKE_ALGAE_L3(ElevatorLevel.STOW, ArmGoals.ALGAE_L3),
  SCORE_CORAL_L1(ElevatorLevel.IDLE, ArmGoals.IDLE_CORAL),
  SCORE_CORAL_L2(ElevatorLevel.L2, ArmGoals.CORAL_L2),
  SCORE_CORAL_L3(ElevatorLevel.STOW, ArmGoals.CORAL_REEF_HIGH),
  SCORE_CORAL_L4(ElevatorLevel.L4, ArmGoals.CORAL_REEF_HIGH),
  SCORE_ALGAE_NET(ElevatorLevel.NET, ArmGoals.NET);

  public ElevatorLevel elevatorGoal;
  public ArmGoals armGoal;
}
