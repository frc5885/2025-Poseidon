package frc.robot.subsystems.SuperStructure;

import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.WristGoals;

public enum SuperStructureState {
  INTAKE_CORAL(ElevatorLevel.L1, ArmGoals.INTAKE, WristGoals.INTAKE), // good
  INTAKE_ALGAE_FLOOR(ElevatorLevel.L1, ArmGoals.INTAKE, WristGoals.ALGAE_FLOOR), // moderate
  INTAKE_ALGAE_LOW_REEF(
      ElevatorLevel.L2ALGAE, ArmGoals.ALGAE_LOW_REEF, WristGoals.ALGAE_REEF), // good
  INTAKE_ALGAE_HIGH_REEF(
      ElevatorLevel.L3ALGAE, ArmGoals.ALGAE_HIGH_REEF, WristGoals.ALGAE_REEF), // good
  SCORE_CORAL_L1(ElevatorLevel.L1, ArmGoals.ALGAE_LOW_REEF, WristGoals.REEF), // good
  SCORE_CORAL_L2(ElevatorLevel.L2, ArmGoals.ALGAE_LOW_REEF, WristGoals.REEF), // good
  SCORE_CORAL_L3(ElevatorLevel.L3, ArmGoals.ALGAE_LOW_REEF, WristGoals.REEF), // good
  SCORE_CORAL_L4(ElevatorLevel.L4, ArmGoals.ALGAE_LOW_REEF, WristGoals.L4REEF), // good
  SCORE_ALGAE_PROCESSOR(ElevatorLevel.L1, ArmGoals.INTAKE, WristGoals.PROCESSOR), // moderate
  SCORE_ALGAE_NET(ElevatorLevel.L4, ArmGoals.STOW, WristGoals.REEF), // good
  DEFAULT(
      ElevatorLevel.L1,
      ArmGoals.STOW,
      WristGoals.STOW); // TODO MUST watchout for default state retaining

  public ElevatorLevel elevatorGoal;
  public ArmGoals armGoal;
  public WristGoals wristGoal;

  private SuperStructureState(ElevatorLevel elevatorGoal, ArmGoals armGoal, WristGoals wristGoal) {
    this.elevatorGoal = elevatorGoal;
    this.armGoal = armGoal;
    this.wristGoal = wristGoal;
  }
}
