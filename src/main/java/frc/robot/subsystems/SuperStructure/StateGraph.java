package frc.robot.subsystems.SuperStructure;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class StateGraph {
  private static StateGraph instance;

  private final Map<SuperStructureState, List<SuperStructureState>> adjacencyList = new HashMap<>();

  public static StateGraph getInstance() {
    if (instance == null) {
      instance = initializeGraph();
    }
    return instance;
  }

  /**
   * Add a state to the graph, if it isnt already there
   *
   * @param state node to add
   */
  public void addState(SuperStructureState state) {
    adjacencyList.putIfAbsent(state, new ArrayList<>());
  }

  /**
   * adds a valid transition between two states to the graph note that this is NOT two directional
   *
   * @param from parent state
   * @param to child state
   */
  public void addTransition(SuperStructureState from, SuperStructureState to) {
    adjacencyList.get(from).add(to);
  }

  // Get neighbors (valid next states)
  public List<SuperStructureState> getNeighbors(SuperStructureState state) {
    return adjacencyList.getOrDefault(state, new ArrayList<>());
  }

  /**
   * creates a java representation of the graph of all posible transitions outlined here:
   * {@linkplain
   * https://www.chiefdelphi.com/t/team-177-429-bobcat-robotics-program-2025-build-blog/478233/62}
   */
  public static StateGraph initializeGraph() {
    // initialize empty graph
    StateGraph graph = new StateGraph();

    // Add all states to the graph
    // this creates the nodes (potential states),
    // we then add the edges (valid transitions between states) later
    for (SuperStructureState state : SuperStructureState.values()) {
      graph.addState(state);
    }

    // create list of all possible transitions
    // first element in list is the parent, the following elements
    // are all the children, or possible transitions from the parent state
    List<SuperStructureState[]> transitions =
        List.of(
            // INTAKE_CORAL transitions
            new SuperStructureState[] {
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.TRANSITION_FOR_STATION,
              SuperStructureState.IDLE,
              SuperStructureState.INTAKE_ALGAE_FLOOR,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.SCORE_CORAL_L4,
              SuperStructureState.SCORE_ALGAE_PROCESSOR,
              SuperStructureState.STOWING,
            },

            // CORAL_STATION transitions
            new SuperStructureState[] {
              SuperStructureState.CORAL_STATION, SuperStructureState.TRANSITION_FOR_STATION,
            },

            // IDLE transitions
            new SuperStructureState[] {
              SuperStructureState.IDLE,
              SuperStructureState.IDLE_TO_INTAKE,
              SuperStructureState.STOWING
            },

            // IDLE_TO_INTAKE transitions
            new SuperStructureState[] {
              SuperStructureState.IDLE_TO_INTAKE, SuperStructureState.INTAKE_CORAL,
            },

            // INTAKE_ALGAE_FLOOR transitions
            new SuperStructureState[] {
              SuperStructureState.INTAKE_ALGAE_FLOOR,
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.TRANSITION_FOR_STATION,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.SCORE_CORAL_L4,
              SuperStructureState.SCORE_ALGAE_PROCESSOR,
            },

            // INTAKE_ALGAE_L2 transitions
            new SuperStructureState[] {
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.TRANSITION_FOR_STATION,
              SuperStructureState.INTAKE_ALGAE_FLOOR,
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.SCORE_CORAL_L4,
              SuperStructureState.SCORE_ALGAE_PROCESSOR,
            },

            // INTAKE_ALGAE_L3 transitions
            new SuperStructureState[] {
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.TRANSITION_FOR_STATION,
              SuperStructureState.INTAKE_ALGAE_FLOOR,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.SCORE_CORAL_L4,
              SuperStructureState.SCORE_ALGAE_PROCESSOR,
            },

            // SCORE_CORAL_L1 transitions
            new SuperStructureState[] {
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.TRANSITION_FOR_STATION,
              SuperStructureState.INTAKE_ALGAE_FLOOR,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.SCORE_CORAL_L4,
              SuperStructureState.SCORE_ALGAE_PROCESSOR,
            },

            // SCORE_CORAL_L2 transitions
            new SuperStructureState[] {
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.TRANSITION_FOR_STATION,
              SuperStructureState.INTAKE_ALGAE_FLOOR,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.SCORE_CORAL_L4,
              SuperStructureState.SCORE_ALGAE_PROCESSOR,
            },

            // SCORE_CORAL_L3 transitions
            new SuperStructureState[] {
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.TRANSITION_FOR_STATION,
              SuperStructureState.INTAKE_ALGAE_FLOOR,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L4,
              SuperStructureState.SCORE_ALGAE_PROCESSOR,
            },

            // SCORE_CORAL_L4 transitions
            new SuperStructureState[] {
              SuperStructureState.SCORE_CORAL_L4,
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.TRANSITION_FOR_STATION,
              SuperStructureState.INTAKE_ALGAE_FLOOR,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.SCORE_ALGAE_PROCESSOR,
              SuperStructureState.SCORE_ALGAE_NET
            },

            // SCORE_ALGAE_PROCESSOR transitions
            new SuperStructureState[] {
              SuperStructureState.SCORE_ALGAE_PROCESSOR,
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.TRANSITION_FOR_STATION,
              SuperStructureState.INTAKE_ALGAE_FLOOR,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.SCORE_CORAL_L4,
            },

            // SCORE_ALGAE_NET transitions
            new SuperStructureState[] {
              SuperStructureState.SCORE_ALGAE_NET, SuperStructureState.SCORE_CORAL_L4,
            },

            // STOWED transitions
            new SuperStructureState[] {SuperStructureState.STOWED, SuperStructureState.UNSTOWING},

            // STOWING transitions
            new SuperStructureState[] {
              SuperStructureState.STOWING, SuperStructureState.STOWED, SuperStructureState.UNSTOWING
            },

            // UNSTOWING transitions
            new SuperStructureState[] {
              SuperStructureState.UNSTOWING,
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.INTAKE_ALGAE_FLOOR,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.STOWING
            },

            // TRANSITION_FOR_STATION transitions
            new SuperStructureState[] {
              SuperStructureState.TRANSITION_FOR_STATION,
              SuperStructureState.CORAL_STATION,
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.INTAKE_ALGAE_FLOOR,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.SCORE_CORAL_L4,
              SuperStructureState.SCORE_ALGAE_PROCESSOR,
            });

    // for each set of transitions
    for (SuperStructureState[] transition : transitions) {
      SuperStructureState parent = transition[0];
      // for each transition in the set
      for (int i = 1; i < transition.length; i++) {
        // add the edge to the graph
        graph.addTransition(parent, transition[i]);
      }
    }

    return graph;
  }
}
