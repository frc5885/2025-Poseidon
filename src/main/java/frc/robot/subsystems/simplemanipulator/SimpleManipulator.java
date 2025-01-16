// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simplemanipulator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.simplemanipulator.elevator.Elevator;
import frc.robot.subsystems.simplemanipulator.elevator.ElevatorIO;
import org.littletonrobotics.junction.Logger;

public class SimpleManipulator extends SubsystemBase {
  private final Elevator m_elevator;

  private SysIdRoutine m_sysIdRoutine;

  public SimpleManipulator(ElevatorIO io) {
    m_elevator = new Elevator(io);

    // Configure SysId
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SimpleManipulator/Elevator", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    m_elevator.periodic();

    m_elevator.runElevatorSetpoint(12.0);
  }

  // TODO May adjust limits to avoid damaging the mechanism
  public void runCharacterization(double outputVolts) {
    m_elevator.runCharacterization(outputVolts);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(m_sysIdRoutine.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(m_sysIdRoutine.dynamic(direction));
  }
}
