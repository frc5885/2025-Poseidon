// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SuperStructure.Arm.Arm;
import frc.robot.subsystems.SuperStructure.Arm.ArmIO;
import frc.robot.subsystems.SuperStructure.Elevator.Elevator;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIO;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import org.littletonrobotics.junction.AutoLogOutput;

public class SuperStructure extends SubsystemBase {
  private final Elevator m_elevator;
  private final Arm m_arm;

  public SuperStructure(ElevatorIO elevatorIO, ArmIO armIO) {
    m_elevator = new Elevator(elevatorIO);
    m_arm = new Arm(armIO);

    m_elevator.sysIdSetup(this);
    m_arm.sysIdSetup(this);
  }

  @Override
  public void periodic() {
    m_elevator.periodic();
    m_arm.periodic();
  }

  @AutoLogOutput(key = "SuperStructure/Elevator/Level")
  public ElevatorLevel getElevatorLevel() {
    return m_elevator.getLevel();
  }

  @AutoLogOutput(key = "SuperStructure/Arm/Goal")
  public ArmGoals getArmGoal() {
    return m_arm.getGoal();
  }

  public void setElevatorLevel(ElevatorLevel elevatorLevel) {
    m_elevator.setLevel(elevatorLevel);
  }

  public void setArmGoal(ArmGoals armGoal) {
    m_arm.setGoal(armGoal);
  }

  // used to determine if the superstructure achieved the combined([elevator, arm]) goal state
  @AutoLogOutput(key = "SuperStructure/isGoalAchieved")
  public boolean isGoalAchieved() {
    return m_elevator.isSetpointAchieved() && m_arm.isSetpointAchieved();
  }

  /** Returns a command to run a elevator quasistatic test in the specified direction. */
  public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_elevator.getSysIdQuasistatic(direction);
  }

  /** Returns a command to run a elevator dynamic test in the specified direction. */
  public Command elevatorSysIdDynamic(SysIdRoutine.Direction direction) {
    return m_elevator.getSysIdDynamic(direction);
  }

  /** Returns a command to run a arm quasistatic test in the specified direction. */
  public Command armSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_arm.getSysIdQuasistatic(direction);
  }

  /** Returns a command to run a arm dynamic test in the specified direction. */
  public Command armSysIdDynamic(SysIdRoutine.Direction direction) {
    return m_arm.getSysIdDynamic(direction);
  }
}
