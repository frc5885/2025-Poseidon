// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import static edu.wpi.first.units.Units.*;

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
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private final Elevator m_elevator;
  private final Arm m_arm;

  private SysIdRoutine m_elevatorSysIdRoutine;

  public SuperStructure(ElevatorIO elevatorIO, ArmIO armIO) {
    m_elevator = new Elevator(elevatorIO);
    m_arm = new Arm(armIO);

    // Configure SysId
    m_elevatorSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput("SuperStructure/ElevatorSysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runElevatorCharacterization(voltage.in(Volts)), null, this));
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

  // TODO May adjust limits to avoid damaging the mechanism
  public void runElevatorCharacterization(double outputVolts) {
    m_elevator.runCharacterization(outputVolts);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runElevatorCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(m_elevatorSysIdRoutine.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command elevatorSysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runElevatorCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(m_elevatorSysIdRoutine.dynamic(direction));
  }
}
