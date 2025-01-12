// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simplemanipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.simplemanipulator.elevator.Elevator;
import frc.robot.subsystems.simplemanipulator.elevator.ElevatorIO;

public class SimpleManipulator extends SubsystemBase {
  private final Elevator m_elevator;

  public SimpleManipulator(ElevatorIO io) {
    m_elevator = new Elevator(io);
  }

  @Override
  public void periodic() {
    m_elevator.periodic();

    m_elevator.runElevatorSetpoint(12.0);
  }
}
