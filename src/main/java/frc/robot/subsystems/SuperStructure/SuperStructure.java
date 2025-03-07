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
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperStructure extends SubsystemBase {
  private final Elevator m_elevator;
  private final Arm m_arm;

  private SuperStructureState m_state = SuperStructureState.STOWED;
  private SuperStructureState m_finalGoal = SuperStructureState.STOWED;
  private StateGraph m_graph = StateGraph.getInstance();

  private LoggedMechanism2d m_canvas;
  private LoggedMechanismRoot2d m_elevatorRoot;
  private LoggedMechanismRoot2d m_carriageRoot;
  private LoggedMechanismRoot2d m_armRoot;
  private LoggedMechanismLigament2d m_armMech;

  private double m_canvasWidth = 3.0;
  private Translation2d m_armRootTranslation;

  private Runnable m_extendIntakeCmd = null;
  private Runnable m_retractIntakeCmd = null;

  public SuperStructure(ElevatorIO elevatorIO, ArmIO armIO, BooleanSupplier disablePIDs) {
    m_elevator = new Elevator(elevatorIO, disablePIDs);
    m_arm = new Arm(armIO, disablePIDs);

    visualizationSetup();

    m_elevator.sysIdSetup(this);
    m_arm.sysIdSetup(this);
  }

  @Override
  public void periodic() {
    m_elevator.periodic();
    m_arm.periodic();

    visualizationUpdate();
  }

  @AutoLogOutput(key = "SuperStructure/Elevator/Goal")
  public ElevatorLevel getElevatorGoal() {
    ElevatorLevel goal = m_elevator.getGoal();
    Logger.recordOutput("SuperStructure/Elevator/GoalPosition", goal.setpointMeters);
    Logger.recordOutput(
        "SuperStructure/Elevator/SetPointPosition", m_elevator.getSetpointRadians());
    return goal;
  }

  public DoubleSupplier getAdjustmentCoefficient() {
    return m_elevator::getAdjustmentCoefficient;
  }

  @AutoLogOutput(key = "SuperStructure/Arm/Goal")
  public ArmGoals getArmGoal() {
    ArmGoals goal = m_arm.getGoal();
    Logger.recordOutput(
        "SuperStructure/Arm/GoalPosition",
        Units.degreesToRadians(goal.setpointDegrees.getAsDouble()));
    Logger.recordOutput("SuperStructure/Arm/SetPointPosition", m_arm.getSetpointRadians());
    return goal;
  }

  @AutoLogOutput(key = "SuperStructure/Goal")
  public SuperStructureState getSuperStructureGoal() {
    return m_state;
  }

  public void setIntakeFunctions(Runnable extendIntake, Runnable retractIntake) {
    m_extendIntakeCmd = extendIntake;
    m_retractIntakeCmd = retractIntake;
  }

  public SequentialCommandGroup setSuperStructureGoal(SuperStructureState state) {
    m_finalGoal = state;
    Logger.recordOutput("SuperStructure/FinalGoal", m_finalGoal);
    List<SuperStructureState> states = findShortestPath(getSuperStructureGoal(), state);
    Logger.recordOutput("SuperStructure/States", states.toString());

    return new SequentialCommandGroup(
        states.stream()
            .filter(desiredState -> desiredState != m_state)
            .map(this::setSingleState)
            .toArray(Command[]::new));
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
    // Null defense: if start or goal is null, return an empty list.
    if (start == null || goal == null) {
      return Collections.emptyList();
    }

    // If start equals goal, immediately return a singleton list.
    if (start.equals(goal)) {
      return Collections.singletonList(start);
    }

    // Use a queue for breadth-first search (BFS) where each element is a path (list of states).
    Queue<List<SuperStructureState>> queue = new LinkedList<>();
    // Use a set to track visited states to avoid cycles and redundant paths.
    Set<SuperStructureState> visited = new HashSet<>();

    // Start the BFS with a path that only contains the start state.
    List<SuperStructureState> initialPath = new ArrayList<>();
    initialPath.add(start);
    queue.add(initialPath);
    visited.add(start);

    while (!queue.isEmpty()) {
      // Dequeue the next path to explore.
      List<SuperStructureState> path = queue.poll();
      SuperStructureState lastState = path.get(path.size() - 1);

      // If the last state is the goal, we've found the shortest path.
      if (lastState.equals(goal)) {
        return path;
      }

      // Explore each neighbor of the last state.
      for (SuperStructureState neighbor : m_graph.getNeighbors(lastState)) {
        // Only consider neighbors that haven't been visited.
        if (!visited.contains(neighbor)) {
          visited.add(neighbor);
          // Create a new path that extends the current path with the neighbor.
          List<SuperStructureState> newPath = new ArrayList<>(path);
          newPath.add(neighbor);
          queue.add(newPath);
        }
      }
    }

    // If no path is found, return an empty list instead of null.
    return Collections.emptyList();
  }

  private Command setSingleState(SuperStructureState goal) {
    // If goal is either STOWING or UNSTOWING, run the intake commands around the state command.
    if (goal == SuperStructureState.STOWING || goal == SuperStructureState.UNSTOWING) {
      // Only retract if the final goal is not INTAKE_CORAL
      boolean shouldRetract = m_finalGoal != SuperStructureState.INTAKE_CORAL;

      return Commands.sequence(
          runIfNotNull(m_extendIntakeCmd),
          createStateCommand(goal),
          shouldRetract ? runIfNotNull(m_retractIntakeCmd) : Commands.none());
    }
    // Otherwise, just run the state command.
    return createStateCommand(goal);
  }

  // Helper that returns a runOnce command if the provided command is non-null, or a no-op command.
  private Command runIfNotNull(Runnable cmd) {
    return cmd != null ? Commands.runOnce(cmd::run) : Commands.none();
  }

  // Creates a command that sets the elevator, arm, and wrist to the specified goal state.
  private Command createStateCommand(SuperStructureState goal) {
    return Commands.run(
            () -> {
              m_elevator.setGoal(goal.elevatorGoal);
            },
            this)
        .until(this::isGoalAchieved)
        .finallyDo(() -> m_state = goal);
  }

  // used to determine if the superstructure achieved the combined([elevator, arm, wrist]) goal
  // state
  @AutoLogOutput(key = "SuperStructure/isGoalAchieved")
  public boolean isGoalAchieved() {
    return m_elevator.isSetpointAchieved() && m_arm.isSetpointAchieved();
  }

  @AutoLogOutput(key = "SuperStructure/isFinalGoalAchieved")
  public boolean isFinalGoalAchieved() {
    return m_state == m_finalGoal && isGoalAchieved();
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

  public void runElevatorOpenLoop(double voltage) {
    m_elevator.runElevatorOpenLoop(voltage);
  }

  public void runArmOpenLoop(double voltage) {
    m_arm.runArmOpenLoop(voltage);
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
  }

  private void visualizationUpdate() {
    // update mechanism 2d
    m_carriageRoot.setPosition(
        m_canvasWidth / 2 + kElevatorTranslation.getX() + 0.05,
        0.15 + m_elevator.getPositionMeters());
    m_armRoot.setPosition(
        m_armRootTranslation.getX(), m_armRootTranslation.getY() + m_elevator.getPositionMeters());
    m_armMech.setAngle(Units.radiansToDegrees(m_arm.getPositionRadians()));
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
  }

  /* Returns the pose of the end effector for game piece visualization */
  public Pose3d getEndEffectorPose3d() {
    return new Pose3d(
        kElevatorTranslation.getX()
            + 0.06
            + kArmLengthMeters * Math.cos(Units.degreesToRadians(m_armMech.getAngle())),
        0,
        m_armRootTranslation.getY()
            + m_elevator.getPositionMeters()
            + kArmLengthMeters * Math.sin(Units.degreesToRadians(m_armMech.getAngle())),
        new Rotation3d());
  }
}
