package frc.robot.subsystems.SuperStructure;

import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public enum SuperStructureState {
  // STOWED(ElevatorLevel.STOW, ArmGoals.STOW),
  // IDLE_CORAL(ElevatorLevel.IDLE, ArmGoals.IDLE_CORAL),
  IDLE(ElevatorLevel.IDLE, ArmGoals.IDLE),
  IDLE_ALGAE(ElevatorLevel.STOW, ArmGoals.IDLE_ALGAE),
  INTAKE_CORAL(ElevatorLevel.STOW, ArmGoals.IDLE),
  INTAKE_ALGAE_L2(ElevatorLevel.STOW, ArmGoals.ALGAE_L2),
  AFTER_ALGAE_L2(ElevatorLevel.AFTER_ALGAE_L2, ArmGoals.ALGAE_L2),
  INTAKE_ALGAE_L3(ElevatorLevel.ALGAE_L3, ArmGoals.ALGAE_L3),
  AFTER_ALGAE_L3(ElevatorLevel.AFTER_ALGAE_L3, ArmGoals.ALGAE_L3),
  SCORE_CORAL_L1(ElevatorLevel.IDLE, ArmGoals.CORAL_L1),
  SCORE_CORAL_L2(ElevatorLevel.L2, ArmGoals.CORAL_L2),
  SCORE_CORAL_L3(ElevatorLevel.STOW, ArmGoals.CORAL_REEF_HIGH),
  SCORE_CORAL_L4(ElevatorLevel.L4, ArmGoals.CORAL_REEF_HIGH),
  SCORED_CORAL_L2(ElevatorLevel.L2, ArmGoals.SCORED_CORAL_L2),
  SCORED_CORAL_L3(ElevatorLevel.STOW, ArmGoals.SCORED_REEF_HIGH),
  SCORED_CORAL_L4(ElevatorLevel.L4, ArmGoals.SCORED_REEF_HIGH),
  SCORE_ALGAE_NET(ElevatorLevel.NET, ArmGoals.NET),
  SCORE_ALGAE_PROCESSOR(ElevatorLevel.STOW, ArmGoals.PROCESSOR),
  INTAKE_LOLLIPOP(ElevatorLevel.STOW, ArmGoals.ALGAE_LOLLIPOP),

  // transition for between idle and L3
  BEFORE_L3(ElevatorLevel.IDLE, ArmGoals.IDLE),
  BEFORE_NET(ElevatorLevel.NET, ArmGoals.IDLE_ALGAE),

  // climb
  BEFORE_CLIMB(ElevatorLevel.CLIMB, ArmGoals.IDLE),
  CLIMBING(ElevatorLevel.CLIMB, ArmGoals.CLIMB),
  CLIMBED(ElevatorLevel.STOW, ArmGoals.CLIMB);

  public ElevatorLevel elevatorGoal;
  public ArmGoals armGoal;
}
