// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simplemanipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.simplemanipulator.ManipulatorConstants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.simplemanipulator.elevator.Elevator;
import frc.robot.subsystems.simplemanipulator.elevator.ElevatorIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SimpleManipulator extends SubsystemBase {
  private final Elevator m_elevator;

  private SysIdRoutine m_elevatorSysIdRoutine;

  public SimpleManipulator(ElevatorIO io) {
    m_elevator = new Elevator(io);

    // Configure SysId
    m_elevatorSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput("SimpleManipulator/ElevatorSysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runElevatorCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    m_elevator.periodic();
  }

  @AutoLogOutput(key = "SimpleManipulator/Elevator/Level")
  public ElevatorLevel getElevatorLevel() {
    return m_elevator.getLevel();
  }

  public void setElevatorLevel(ElevatorLevel elevatorLevel) {
    m_elevator.setLevel(elevatorLevel);
  }

  // used to determine if the manipulator achieved the combined([elevator, arm]) goal state
  @AutoLogOutput(key = "SimpleManipulator/isGoalAchieved")
  public boolean isGoalAchieved() {
    return m_elevator.isSetpointAchieved();
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
