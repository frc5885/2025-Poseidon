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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.SuperStructure.Arm.Arm;
import frc.robot.subsystems.SuperStructure.Arm.ArmIO;
import frc.robot.subsystems.SuperStructure.Elevator.Elevator;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIO;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.WristGoals;
import frc.robot.subsystems.SuperStructure.Wrist.Wrist;
import frc.robot.subsystems.SuperStructure.Wrist.WristIO;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperStructure extends SubsystemBase {
  private final Elevator m_elevator;
  private final Arm m_arm;
  private final Wrist m_wrist;

  private SuperStructureState m_state = SuperStructureState.STOWED;
  private StateGraph m_graph = StateGraph.getInstance();

  private LoggedMechanism2d m_canvas;
  private LoggedMechanismRoot2d m_elevatorRoot;
  private LoggedMechanismRoot2d m_carriageRoot;
  private LoggedMechanismRoot2d m_armRoot;
  private LoggedMechanismLigament2d m_armMech;
  private LoggedMechanismLigament2d m_wristMech;

  private double m_canvasWidth = 3.0;
  private Translation2d m_armRootTranslation;

  public SuperStructure(ElevatorIO elevatorIO, ArmIO armIO, WristIO wristIO) {
    m_elevator = new Elevator(elevatorIO);
    m_arm = new Arm(armIO);
    m_wrist = new Wrist(wristIO);
    m_wrist.setArmAngleSupplier(m_arm::getPositionRadians);

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
    ArmGoals goal = m_arm.getGoal();
    Logger.recordOutput("SuperStructure/Arm/GoalPosition", goal.setpointDegrees);
    return goal;
  }

  @AutoLogOutput(key = "SuperStructure/Wrist/Goal")
  public WristGoals getWristGoal() {
    WristGoals goal = m_wrist.getGoal();
    Logger.recordOutput("SuperStructure/Wrist/GoalPosition", goal.setpointDegrees);
    return goal;
  }

  public void setElevatorLevel(ElevatorLevel elevatorLevel) {
    m_elevator.setLevel(elevatorLevel);
  }

  public void setArmGoal(ArmGoals armGoal) {
    m_arm.setGoal(armGoal);
  }

  public void setWristGoal(WristGoals wristGoal) {
    m_wrist.setGoal(wristGoal);
  }

  @AutoLogOutput(key = "SuperStructure/Goal")
  public SuperStructureState getSuperStructureGoal() {
    return m_state;
  }

  public SequentialCommandGroup setSuperStructureGoal(SuperStructureState state) {
    List<SuperStructureState> states = findShortestPath(getSuperStructureGoal(), state);
    Logger.recordOutput("SuperStructure/States", states.toString());

    SequentialCommandGroup result = new SequentialCommandGroup();

    for (SuperStructureState desiredState : states) {
      if (desiredState != m_state) {
        result.addCommands(setSingleState(desiredState));
      }
    }
    return result;
  }

  /**
   * Performs a breadth-first search of all possible states to find the shortest path from the start
   * state to the goal state.
   *
   * @param start Current state
   * @param goal desired goal state
   * @return A list of states representing the shortest path from start to goal, {@code null} if no
   *     path exists.
   */
  public List<SuperStructureState> findShortestPath(
      SuperStructureState start, SuperStructureState goal) {

    // Queue to manage the paths to be explored. Each element in the queue is a list of states
    // representing a path.
    Queue<List<SuperStructureState>> queue = new LinkedList<>();

    // Set to keep track of visited states to avoid revisiting them and getting stuck in loops.
    Set<SuperStructureState> visited = new HashSet<>();

    // Start BFS with a path containing only the start node.
    // The initial path is a singleton list containing just the start state.
    queue.add(Collections.singletonList(start));

    // Continue exploring until there are no more paths to explore in the queue.
    while (!queue.isEmpty()) {
      // Retrieve and remove the first path from the queue.
      List<SuperStructureState> path = queue.poll();

      // Get the last state in the current path. This is the state we will explore next.
      SuperStructureState lastState = path.get(path.size() - 1);

      // Check if the last state in the current path is the goal state.
      if (lastState.equals(goal)) {
        // If it is, return the current path as it is the shortest path found.
        return path;
      }

      // If the last state has not been visited yet, process it.
      if (!visited.contains(lastState)) {
        // Mark the last state as visited to avoid processing it again in the future.
        visited.add(lastState);

        // Explore all neighboring states of the last state.
        for (SuperStructureState neighbor : m_graph.getNeighbors(lastState)) {
          // Create a new path by copying the current path and adding the neighbor state to it.
          List<SuperStructureState> newPath = new ArrayList<>(path);
          newPath.add(neighbor);

          // Add the new path to the queue for further exploration.
          queue.add(newPath);
        }
      }
    }

    // If the queue is exhausted and no path to the goal state is found, return null.
    return null;
  }

  private Command setSingleState(SuperStructureState goal) {
    return Commands.run(
            () -> {
              m_elevator.setLevel(goal.elevatorGoal);
              m_arm.setGoal(goal.armGoal);
              m_wrist.setGoal(goal.wristGoal);
            },
            this)
        .until(() -> isGoalAchieved())
        .finallyDo(
            () -> {
              m_state = goal;
            });
  }

  // used to determine if the superstructure achieved the combined([elevator, arm, wrist]) goal
  // state
  @AutoLogOutput(key = "SuperStructure/isGoalAchieved")
  public boolean isGoalAchieved() {
    return m_elevator.isSetpointAchieved()
        && m_arm.isSetpointAchieved()
        && m_wrist.isSetpointAchieved();
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
    m_wristMech =
        m_armMech.append(
            new LoggedMechanismLigament2d(
                "Wrist",
                kWristLengthMeters,
                Units.radiansToDegrees(kWristStartingPositionRadians),
                10.0,
                new Color8Bit(0, 255, 255)));
    // parts of algae claw (don't need to update because they're fixed relative to wrist)
    m_wristMech.append(
        new LoggedMechanismLigament2d(
            "AlgaeClaw1", kWristLengthMeters, -149, 10.0, new Color8Bit(0, 255, 255)));
    m_wristMech.append(
        new LoggedMechanismLigament2d(
            "AlgaeClaw2", kWristLengthMeters, -84, 10.0, new Color8Bit(0, 255, 255)));
  }

  private void visualizationUpdate() {
    // update mechanism 2d
    m_carriageRoot.setPosition(
        m_canvasWidth / 2 + kElevatorTranslation.getX() + 0.05,
        0.15 + m_elevator.getPositionMeters());
    m_armRoot.setPosition(
        m_armRootTranslation.getX(), m_armRootTranslation.getY() + m_elevator.getPositionMeters());
    m_armMech.setAngle(Units.radiansToDegrees(m_arm.getPositionRadians()));
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
            kElevatorTranslation.getX()
                + 0.06
                + kArmLengthMeters * Math.cos(Units.degreesToRadians(m_armMech.getAngle())),
            0,
            m_armRootTranslation.getY()
                + m_elevator.getPositionMeters()
                + kArmLengthMeters * Math.sin(Units.degreesToRadians(m_armMech.getAngle())),
            new Rotation3d(0.0, -m_wrist.getRealWorldPositionRadians(), 0.0)));
  }
}
