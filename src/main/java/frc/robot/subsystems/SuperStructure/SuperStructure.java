// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SuperStructure;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.SuperStructure.Arm.Arm;
import frc.robot.subsystems.SuperStructure.Arm.ArmIO;
import frc.robot.subsystems.SuperStructure.Elevator.Elevator;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIO;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.SuperStructure.Wrist.Wrist;
import frc.robot.subsystems.SuperStructure.Wrist.WristIO;
import frc.robot.subsystems.SuperStructure.Wrist.WristIOSim;
import frc.robot.subsystems.SuperStructure.Wrist.WristIOSpark;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperStructure extends SubsystemBase {
  private final Elevator m_elevator;
  private final Arm m_arm;
  private final Wrist m_wrist;

  private LoggedMechanism2d m_canvas;
  private LoggedMechanismRoot2d m_elevatorRoot;
  private LoggedMechanismRoot2d m_carriageRoot;
  private LoggedMechanismRoot2d m_armRoot;
  private LoggedMechanismLigament2d m_armMech;
  private LoggedMechanismRoot2d m_wristRoot;
  private LoggedMechanismLigament2d m_wristMech;

  private double m_canvasWidth = 3.0;
  private Translation2d m_armRootTranslation;

  public SuperStructure(ElevatorIO elevatorIO, ArmIO armIO) {
    m_elevator = new Elevator(elevatorIO);
    m_arm = new Arm(armIO);
    switch (Constants.kCurrentMode) {
      case REAL:
        m_wrist = new Wrist(new WristIOSpark(m_arm::getPositionRadians));
        break;
      case SIM:
        m_wrist = new Wrist(new WristIOSim(m_arm::getPositionRadians));
        break;
      default:
        m_wrist = new Wrist(new WristIO() {});
        break;
    }

    visualizationSetup();

    m_elevator.sysIdSetup(this);
    m_arm.sysIdSetup(this);
    m_wrist.sysIdSetup(this);
  }

  @Override
  public void periodic() {
    m_elevator.periodic();
    m_arm.periodic();
    m_wrist.periodic();

    visualizationUpdate();
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

  /** Returns a command to run a wrist quasistatic test in the specified direction. */
  public Command wristSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_wrist.getSysIdQuasistatic(direction);
  }

  /** Returns a command to run a wrist dynamic test in the specified direction. */
  public Command wristSysIdDynamic(SysIdRoutine.Direction direction) {
    return m_wrist.getSysIdDynamic(direction);
  }

  private void visualizationSetup() {
    m_canvas = new LoggedMechanism2d(m_canvasWidth, 3.0);
    m_elevatorRoot =
        m_canvas.getRoot("ElevatorRoot", m_canvasWidth / 2 + kElevatorTranslation.getX(), 0.15);
    m_elevatorRoot.append(
        new LoggedMechanismLigament2d(
            "Elevator", kElevatorMaxHeightMeters + kElevatorCarriageHeight, 90.0));
    m_carriageRoot =
        m_canvas.getRoot(
            "CarriageRoot", m_canvasWidth / 2 + kElevatorTranslation.getX() + 0.05, 0.15);
    m_carriageRoot.append(
        new LoggedMechanismLigament2d(
            "Carriage", kElevatorCarriageHeight, 90.0, 10.0, new Color8Bit(255, 0, 0)));
    m_armRootTranslation =
        new Translation2d(
            m_canvasWidth / 2 + kElevatorTranslation.getX() + 0.05,
            0.15 + kElevatorCarriageHeight / 2);
    m_armRoot =
        m_canvas.getRoot(
            "ArmRoot",
            m_canvasWidth / 2 + kElevatorTranslation.getX() + 0.05,
            0.15 + kElevatorCarriageHeight / 2);
    m_armMech =
        m_armRoot.append(
            new LoggedMechanismLigament2d(
                "Arm",
                kArmLengthMeters,
                Units.radiansToDegrees(kArmStartingPositionRadians),
                10.0,
                new Color8Bit(0, 255, 0)));
    m_wristRoot =
        m_canvas.getRoot(
            "WristRoot",
            m_armRootTranslation.getX()
                + kArmLengthMeters * Math.cos(Units.degreesToRadians(m_armMech.getAngle())),
            m_armRootTranslation.getY()
                + kArmLengthMeters * Math.sin(Units.degreesToRadians(m_armMech.getAngle())));
    m_wristMech =
        m_armRoot.append(
            new LoggedMechanismLigament2d(
                "Wrist",
                kWristLengthMeters,
                Units.radiansToDegrees(kWristStartingPositionRadians),
                10.0,
                new Color8Bit(255, 0, 0)));
  }

  private void visualizationUpdate() {
    // update mechanism 2d
    m_carriageRoot.setPosition(
        m_canvasWidth / 2 + kElevatorTranslation.getX() + 0.05,
        0.15 + m_elevator.getPositionMeters());
    m_armRoot.setPosition(
        m_armRootTranslation.getX(), m_armRootTranslation.getY() + m_elevator.getPositionMeters());
    m_armMech.setAngle(Units.radiansToDegrees(m_arm.getPositionRadians()));
    m_wristRoot.setPosition(
        m_armRootTranslation.getX()
            + kArmLengthMeters * Math.cos(Units.degreesToRadians(m_armMech.getAngle())),
        m_armRootTranslation.getY()
            + kArmLengthMeters * Math.sin(Units.degreesToRadians(m_armMech.getAngle())));
    m_wristMech.setAngle(Units.radiansToDegrees(m_wrist.getPositionRadians()));
    Logger.recordOutput("SuperStructure/Mechanism2d", m_canvas);

    // Log pose 3d
    Logger.recordOutput(
        "SuperStructure/Mechanism3d/0-ElevatorStage1",
        new Pose3d(
            0.0,
            0.0,
            m_elevator.getPositionMeters() * kElevatorStage1MaxTravel / kElevatorMaxHeightMeters,
            new Rotation3d()));
    Logger.recordOutput(
        "SuperStructure/Mechanism3d/1-ElevatorCarriage",
        new Pose3d(0.0, 0.0, m_elevator.getPositionMeters(), new Rotation3d()));
    Logger.recordOutput(
        "SuperStructure/Mechanism3d/2-Arm",
        new Pose3d(
            kElevatorTranslation.getX() + 0.06,
            0,
            m_armRootTranslation.getY() + m_elevator.getPositionMeters(),
            new Rotation3d(0, -m_arm.getPositionRadians(), 0)));
    Logger.recordOutput(
        "SuperStructure/Mechanism3d/3-Wrist",
        new Pose3d(
            m_armRootTranslation.getX()
                + 0.06
                + kArmLengthMeters * Math.cos(Units.degreesToRadians(m_armMech.getAngle())),
            0,
            m_armRootTranslation.getY()
                + m_elevator.getPositionMeters()
                + kArmLengthMeters * Math.sin(Units.degreesToRadians(m_armMech.getAngle())),
            new Rotation3d(0.0, -m_wrist.getPositionRadians(), 0.0)));
  }
}
