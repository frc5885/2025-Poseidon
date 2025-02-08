// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperStructure extends SubsystemBase {
  private final Elevator m_elevator;
  private final Arm m_arm;

  private final LoggedMechanism2d m_canvas;
  private final LoggedMechanismRoot2d m_elevatorRoot;
  private final LoggedMechanismRoot2d m_carriageRoot;
  private final LoggedMechanismLigament2d m_armMech;

  public SuperStructure(ElevatorIO elevatorIO, ArmIO armIO) {
    m_elevator = new Elevator(elevatorIO);
    m_arm = new Arm(armIO);

    m_canvas = new LoggedMechanism2d(3.0, 3.0);
    m_elevatorRoot = m_canvas.getRoot("ElevatorRoot", 1.47, 0.15);
    m_elevatorRoot.append(
        new LoggedMechanismLigament2d("Elevator", kElevatorMaxHeightMeters, 90.0));
    m_carriageRoot = m_canvas.getRoot("CarriageRoot", 1.53, 0.15);
    m_carriageRoot.append(
        new LoggedMechanismLigament2d("Carriage", 0.3, 90.0, 10.0, new Color8Bit(255, 0, 0)));
    m_armMech =
        m_carriageRoot.append(
            new LoggedMechanismLigament2d(
                "Arm",
                kArmLengthMeters,
                Units.radiansToDegrees(kArmStartingPositionRadians),
                10.0,
                new Color8Bit(0, 255, 0)));

    m_elevator.sysIdSetup(this);
    m_arm.sysIdSetup(this);
  }

  @Override
  public void periodic() {
    m_elevator.periodic();
    m_arm.periodic();

    m_armMech.setAngle(Units.radiansToDegrees(m_arm.getPositionRadians()));
    m_carriageRoot.setPosition(1.53, 0.15 + m_elevator.getPositionMeters());
    Logger.recordOutput("SuperStructure/Mechanism2d", m_canvas);
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
