package frc.robot.subsystems.SuperStructure;

import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.WristGoals;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public enum SuperStructureState {
  INTAKE_CORAL(ElevatorLevel.STOW, ArmGoals.INTAKE, WristGoals.INTAKE),
  CORAL_STATION(ElevatorLevel.CORAL_STATION, ArmGoals.CORAL_STATION, WristGoals.LOCK),
  INTAKE_ALGAE_FLOOR(ElevatorLevel.L2, ArmGoals.ALGAE_FLOOR, WristGoals.ALGAE_FLOOR),
  INTAKE_ALGAE_L2(ElevatorLevel.ALGAE_L2, ArmGoals.REEF, WristGoals.ALGAE_REEF),
  INTAKE_ALGAE_L3(ElevatorLevel.ALGAE_L3, ArmGoals.REEF, WristGoals.ALGAE_REEF),
  SCORE_CORAL_L1(ElevatorLevel.L1, ArmGoals.REEF, WristGoals.L1REEF),
  SCORE_CORAL_L2(ElevatorLevel.L2, ArmGoals.REEF, WristGoals.REEF),
  SCORE_CORAL_L3(ElevatorLevel.L3, ArmGoals.REEF, WristGoals.REEF),
  SCORE_CORAL_L4(ElevatorLevel.L4, ArmGoals.REEF, WristGoals.L4REEF),
  SCORE_ALGAE_PROCESSOR(ElevatorLevel.STOW, ArmGoals.INTAKE, WristGoals.PROCESSOR),
  SCORE_ALGAE_NET(ElevatorLevel.L4, ArmGoals.STOW, WristGoals.NET),
  STOWED(ElevatorLevel.STOW, ArmGoals.STOW, WristGoals.STOW),
  IDLE(ElevatorLevel.STOW, ArmGoals.IDLE, WristGoals.LOCK),
  // TRANSITION STATES
  // while the arm is swinging in/out of stowed state (to/from intake state), lock the wrist to
  // prevent it from crashing
  STOWING(ElevatorLevel.STOW, ArmGoals.STOW, WristGoals.LOCK),
  UNSTOWING(ElevatorLevel.STOW, ArmGoals.INTAKE, WristGoals.LOCK),
  // to prevent wrist from flopping
  IDLE_TO_INTAKE(ElevatorLevel.STOW, ArmGoals.INTAKE, WristGoals.LOCK),
  TRANSITION_FOR_STATION(ElevatorLevel.CORAL_STATION_TRANSITION, ArmGoals.INTAKE,
  WristGoals.STOW),
  INTAKE_TO_SCORE(ElevatorLevel.STOW, ArmGoals.INTAKE, WristGoals.REEF);

  public ElevatorLevel elevatorGoal;
  public ArmGoals armGoal;
  public WristGoals wristGoal;
}
