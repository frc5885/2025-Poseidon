package frc.robot.subsystems.Superstructure;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.Arm.ArmZone;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class Superstructure {
  public static final Distance ELEVATOR_TOLERANCE = Inches.of(0.5);
  public static final Rotation2d ARM_TOLERANCE = Rotation2d.fromDegrees(1.5);
  StateGraph graph = StateGraph.getInstance();

  private SuperstructureState currentState = SuperstructureState.RIGHT_SIDE_UP_IDLE;
  private Arm arm;
  private Elevator elevator;

  /** A class representing the Arm-Elevator superstructure */
  public Superstructure(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
  }

  public SuperstructureState getState() {
    return currentState;
  }

  public SequentialCommandGroup setState(SuperstructureState state) {
    List<SuperstructureState> states = findShortestPath(getState(), state);
    Logger.recordOutput("states", states.toString());

    SequentialCommandGroup result = new SequentialCommandGroup();

    for (SuperstructureState desiredState : states) {
      if (desiredState != currentState) {
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
  public List<SuperstructureState> findShortestPath(
      SuperstructureState start, SuperstructureState goal) {

    // Queue to manage the paths to be explored. Each element in the queue is a list of states
    // representing a path.
    Queue<List<SuperstructureState>> queue = new LinkedList<>();

    // Set to keep track of visited states to avoid revisiting them and getting stuck in loops.
    Set<SuperstructureState> visited = new HashSet<>();

    // Start BFS with a path containing only the start node.
    // The initial path is a singleton list containing just the start state.
    queue.add(Collections.singletonList(start));

    // Continue exploring until there are no more paths to explore in the queue.
    while (!queue.isEmpty()) {
      // Retrieve and remove the first path from the queue.
      List<SuperstructureState> path = queue.poll();

      // Get the last state in the current path. This is the state we will explore next.
      SuperstructureState lastState = path.get(path.size() - 1);

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
        for (SuperstructureState neighbor : graph.getNeighbors(lastState)) {
          // Create a new path by copying the current path and adding the neighbor state to it.
          List<SuperstructureState> newPath = new ArrayList<>(path);
          newPath.add(neighbor);

          // Add the new path to the queue for further exploration.
          queue.add(newPath);
        }
      }
    }

    // If the queue is exhausted and no path to the goal state is found, return null.
    return null;
  }

  /**
   * @return {@code False} if the elevator is high enough where the arm can't collide with the
   *     intake
   */
  public static boolean checkForArmCollision(ArmZone zone, ElevatorState elevatorState) {
    if (isInIntakeZone(zone)) {
      return elevatorState.heightMeters < Elevator.MIN_HEIGHT_INTAKE_AVOIDANCE.in(Meters);
    } else if (zone == ArmZone.BOTTOM_ZONE) {
      return elevatorState.heightMeters < Elevator.MIN_HEIGHT_BOTTOM_AVOIDANCE.in(Meters);
    } else {
      return false;
    }
  }

  public static boolean checkForArmCollision(ArmState armState, ElevatorState elevatorState) {
    return checkForArmCollision(armState.zone, elevatorState);
  }

  public boolean armInTolerance() {
    return Math.abs(currentState.armState.degrees - arm.getPos().getDegrees())
        < ARM_TOLERANCE.getDegrees();
  }

  public boolean elevatorInTolerance() {
    return Math.abs(currentState.elevatorState.heightMeters - elevator.getPos().baseUnitMagnitude())
        < ARM_TOLERANCE.getDegrees();
  }

  /**
   * @return {@code true} if the arm and elevator are within the tolerances for their current states
   */
  public boolean superstructureInTolerance() {
    return elevatorInTolerance() && armInTolerance();
  }

  /**
   * @return {@code true} if the elevator is at or above the min position where the arm can swing
   *     freely without hitting the intake
   */
  public boolean elevatorAboveIntakeMinimum() {
    return elevator.getPos().baseUnitMagnitude()
        >= Elevator.MIN_HEIGHT_INTAKE_AVOIDANCE.baseUnitMagnitude();
  }

  /**
   * @return {@code true} if the elevator is at or above the min position where the arm can swing
   *     freely without hitting the floor
   *     <p>WARNING: the elevator may still be able to hit the intake at this height
   */
  public boolean elevatorAboveFloorMinimum() {
    return elevator.getPos().baseUnitMagnitude()
        >= Elevator.MIN_HEIGHT_BOTTOM_AVOIDANCE.baseUnitMagnitude();
  }

  /**
   * @param zone
   * @return {@code true} if the given armzone is the coral or algae zone
   */
  public static boolean isInIntakeZone(ArmZone zone) {
    return zone == ArmZone.CORAL_INTAKE || zone == ArmZone.ALGAE_INTAKE;
  }

  public static ArmZone getArmZone(SuperstructureState goal) {
    return goal.armState.zone;
  }

  /** see Assets\Docs\TopUpperLimit.png */
  public static ArmZone getArmZone(Rotation2d position) {
    double deg = position.getDegrees();
    if (deg >= Arm.TOP_LOWER_LIMIT.getDegrees() && deg <= Arm.TOP_UPPER_LIMIT.getDegrees()) {
      return ArmZone.TOP_ZONE;
    } else if (deg > Arm.TOP_UPPER_LIMIT.getDegrees()
        && deg < Arm.BOTTOM_LOWER_LIMIT.getDegrees()) {
      return ArmZone.CORAL_INTAKE;
    } else if (deg > Arm.BOTTOM_LOWER_LIMIT.getDegrees()
        && deg < Arm.BOTTOM_UPPER_LIMIT.getDegrees()) {
      return ArmZone.BOTTOM_ZONE;
    } else {
      return ArmZone.ALGAE_INTAKE;
    }
  }

  public boolean superstructureInTolerance(SuperstructureState goal) {
    return (Math.abs(goal.armState.degrees - arm.getPos().getDegrees())
            < ARM_TOLERANCE.getDegrees())
        && (Math.abs(goal.elevatorState.heightMeters - elevator.getPos().baseUnitMagnitude())
            < ELEVATOR_TOLERANCE.baseUnitMagnitude());
  }

  private Command setSingleState(SuperstructureState goal) {
    return Commands.run(
            () -> {
              arm.setState(goal.armState);
              elevator.setState(goal.elevatorState);
            },
            arm,
            elevator)
        .until(() -> superstructureInTolerance(goal))
        .finallyDo(
            () -> {
              currentState = goal;
            });
  }
}
