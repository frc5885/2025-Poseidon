package frc.robot.subsystems.SuperStructure;

import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.WristGoals;

public enum SuperStructureState {
  DEFAULT(ElevatorLevel.L1, ArmGoals.STOW, WristGoals.STOW), // TODO watch out coming back from L4
  INTAKE_CORAL(ElevatorLevel.L1, ArmGoals.INTAKE, WristGoals.INTAKE),
  INTAKE_ALGAE_FLOOR(ElevatorLevel.L1, ArmGoals.INTAKE, WristGoals.ALGAE_FLOOR),
  INTAKE_ALGAE_LOW_REEF(ElevatorLevel.L2ALGAE, ArmGoals.REEF, WristGoals.ALGAE_REEF),
  INTAKE_ALGAE_HIGH_REEF(ElevatorLevel.L3ALGAE, ArmGoals.REEF, WristGoals.ALGAE_REEF),
  SCORE_CORAL_L1(ElevatorLevel.L1, ArmGoals.REEF, WristGoals.L1REEF),
  SCORE_CORAL_L2(ElevatorLevel.L2, ArmGoals.REEF, WristGoals.REEF),
  SCORE_CORAL_L3(ElevatorLevel.L3, ArmGoals.REEF, WristGoals.REEF),
  SCORE_CORAL_L4(ElevatorLevel.L4, ArmGoals.REEF, WristGoals.L4REEF),
  SCORE_ALGAE_PROCESSOR(ElevatorLevel.L1, ArmGoals.INTAKE, WristGoals.PROCESSOR),
  SCORE_ALGAE_NET(ElevatorLevel.L4, ArmGoals.STOW, WristGoals.NET);

  public ElevatorLevel elevatorGoal;
  public ArmGoals armGoal;
  public WristGoals wristGoal;

  private SuperStructureState(ElevatorLevel elevatorGoal, ArmGoals armGoal, WristGoals wristGoal) {
    this.elevatorGoal = elevatorGoal;
    this.armGoal = armGoal;
    this.wristGoal = wristGoal;
  }
}
