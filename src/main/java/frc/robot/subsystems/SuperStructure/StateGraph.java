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
            // IDLE transitions
            new SuperStructureState[] {
              SuperStructureState.IDLE,
              // SuperStructureState.IDLE_ALGAE,
              SuperStructureState.INTAKE_CORAL,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.BEFORE_L3, // Added transition to BEFORE_L3
              SuperStructureState.SCORE_CORAL_L4,
              // SuperStructureState.SCORED_CORAL_L2,
              // SuperStructureState.SCORED_CORAL_L3,
              // SuperStructureState.SCORED_CORAL_L4,
              // SuperStructureState.SCORE_ALGAE_NET,
              SuperStructureState.INTAKE_LOLLIPOP
            },

            // IDLE_ALGAE transitions
            new SuperStructureState[] {
              SuperStructureState.IDLE_ALGAE,
              SuperStructureState.IDLE,
              // SuperStructureState.INTAKE_CORAL,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              // SuperStructureState.SCORE_CORAL_L1,
              // SuperStructureState.SCORE_CORAL_L2,
              // SuperStructureState.BEFORE_L3,
              // SuperStructureState.SCORE_CORAL_L4,
              // SuperStructureState.SCORED_CORAL_L2,
              // SuperStructureState.SCORED_CORAL_L3,
              // SuperStructureState.SCORED_CORAL_L4,
              // SuperStructureState.SCORE_ALGAE_NET,
              SuperStructureState.BEFORE_NET,
              SuperStructureState.INTAKE_LOLLIPOP
            },

            // BEFORE_L3 transitions
            new SuperStructureState[] {
              SuperStructureState.BEFORE_L3,
              SuperStructureState.IDLE,
              SuperStructureState.SCORE_CORAL_L3,
            },

            // INTAKE_CORAL transitions
            new SuperStructureState[] {
              SuperStructureState.INTAKE_CORAL, SuperStructureState.IDLE,
              // SuperStructureState.IDLE_ALGAE,
              // SuperStructureState.INTAKE_ALGAE_L2,
              // SuperStructureState.INTAKE_ALGAE_L3,
              // SuperStructureState.SCORE_CORAL_L1,
              // SuperStructureState.SCORE_CORAL_L2,
              // SuperStructureState.SCORE_CORAL_L3,
              // SuperStructureState.SCORE_CORAL_L4,
              // SuperStructureState.SCORED_CORAL_L2,
              // SuperStructureState.SCORED_CORAL_L3,
              // SuperStructureState.SCORED_CORAL_L4,
              // SuperStructureState.SCORE_ALGAE_NET,
            },

            // INTAKE_ALGAE_L2 transitions - Modified to only go to AFTER_ALGAE_L2
            new SuperStructureState[] {
              SuperStructureState.INTAKE_ALGAE_L2, SuperStructureState.AFTER_ALGAE_L2,
            },

            // AFTER_ALGAE_L2 transitions - New state with previous INTAKE_ALGAE_L2 destinations
            new SuperStructureState[] {
              SuperStructureState.AFTER_ALGAE_L2,
              SuperStructureState.IDLE_ALGAE,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              // SuperStructureState.SCORE_ALGAE_NET,
              SuperStructureState.BEFORE_NET,
            },

            // INTAKE_ALGAE_L3 transitions - Modified to only go to AFTER_ALGAE_L3
            new SuperStructureState[] {
              SuperStructureState.INTAKE_ALGAE_L3, SuperStructureState.AFTER_ALGAE_L3,
            },

            // AFTER_ALGAE_L3 transitions - New state with previous INTAKE_ALGAE_L3 destinations
            new SuperStructureState[] {
              SuperStructureState.AFTER_ALGAE_L3,
              SuperStructureState.IDLE_ALGAE,
              SuperStructureState.INTAKE_ALGAE_L2,
              SuperStructureState.INTAKE_ALGAE_L3,
              // SuperStructureState.SCORE_ALGAE_NET,
              SuperStructureState.BEFORE_NET,
            },

            // SCORE_CORAL_L1 transitions
            new SuperStructureState[] {
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.IDLE,
              // SuperStructureState.IDLE_ALGAE,
              // SuperStructureState.INTAKE_CORAL,
              // SuperStructureState.INTAKE_ALGAE_L2,
              // SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.SCORE_CORAL_L4,
              // SuperStructureState.SCORED_CORAL_L2,
              // SuperStructureState.SCORED_CORAL_L3,
              // SuperStructureState.SCORED_CORAL_L4,
              // SuperStructureState.SCORE_ALGAE_NET,
            },

            // SCORE_CORAL_L2 transitions
            new SuperStructureState[] {
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.IDLE,
              // SuperStructureState.IDLE_ALGAE,
              // SuperStructureState.INTAKE_CORAL,
              // SuperStructureState.INTAKE_ALGAE_L2,
              // SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.SCORE_CORAL_L4,
              SuperStructureState.SCORED_CORAL_L2,
              // SuperStructureState.SCORED_CORAL_L3,
              // SuperStructureState.SCORED_CORAL_L4,
              // SuperStructureState.SCORE_ALGAE_NET,
            },

            // SCORE_CORAL_L3 transitions
            new SuperStructureState[] {
              SuperStructureState.SCORE_CORAL_L3,
              SuperStructureState.IDLE,
              SuperStructureState.BEFORE_L3, // Added transition back to BEFORE_L3
              // SuperStructureState.IDLE_ALGAE,
              // SuperStructureState.INTAKE_CORAL,
              // SuperStructureState.INTAKE_ALGAE_L2,
              // SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L4,
              // SuperStructureState.SCORED_CORAL_L2,
              SuperStructureState.SCORED_CORAL_L3,
              // SuperStructureState.SCORED_CORAL_L4,
              // SuperStructureState.SCORE_ALGAE_NET,
            },

            // SCORE_CORAL_L4 transitions
            new SuperStructureState[] {
              SuperStructureState.SCORE_CORAL_L4,
              SuperStructureState.IDLE,
              // SuperStructureState.IDLE_ALGAE,
              // SuperStructureState.INTAKE_CORAL,
              // SuperStructureState.INTAKE_ALGAE_L2,
              // SuperStructureState.INTAKE_ALGAE_L3,
              SuperStructureState.SCORE_CORAL_L1,
              SuperStructureState.SCORE_CORAL_L2,
              SuperStructureState.SCORE_CORAL_L3,
              // SuperStructureState.SCORED_CORAL_L2,
              // SuperStructureState.SCORED_CORAL_L3,
              SuperStructureState.SCORED_CORAL_L4,
              // SuperStructureState.SCORE_ALGAE_NET,
            },

            // SCORE_CORAL_L4 transitions
            new SuperStructureState[] {
              SuperStructureState.SCORED_CORAL_L2, SuperStructureState.IDLE,
              // SuperStructureState.IDLE_ALGAE,
              // SuperStructureState.INTAKE_CORAL,
              // SuperStructureState.INTAKE_ALGAE_L2,
              // SuperStructureState.INTAKE_ALGAE_L3,
              // SuperStructureState.SCORE_CORAL_L1,
              // SuperStructureState.SCORE_CORAL_L2,
              // SuperStructureState.SCORE_CORAL_L3,
              // SuperStructureState.SCORE_CORAL_L4,
              // SuperStructureState.SCORED_CORAL_L3,
              // SuperStructureState.SCORED_CORAL_L4,
              // SuperStructureState.SCORE_ALGAE_NET,
            },

            // SCORE_CORAL_L4 transitions
            new SuperStructureState[] {
              SuperStructureState.SCORED_CORAL_L3, SuperStructureState.IDLE,
              // SuperStructureState.IDLE_ALGAE,
              // SuperStructureState.INTAKE_CORAL,
              // SuperStructureState.INTAKE_ALGAE_L2,
              // SuperStructureState.INTAKE_ALGAE_L3,
              // SuperStructureState.SCORE_CORAL_L1,
              // SuperStructureState.SCORE_CORAL_L2,
              // SuperStructureState.SCORE_CORAL_L3,
              // SuperStructureState.SCORE_CORAL_L4,
              // SuperStructureState.SCORED_CORAL_L2,
              // SuperStructureState.SCORED_CORAL_L4,
              // SuperStructureState.SCORE_ALGAE_NET,
            },

            // SCORE_CORAL_L4 transitions
            new SuperStructureState[] {
              SuperStructureState.SCORED_CORAL_L4, SuperStructureState.IDLE,
              // SuperStructureState.IDLE_ALGAE,
              // SuperStructureState.INTAKE_CORAL,
              // SuperStructureState.INTAKE_ALGAE_L2,
              // SuperStructureState.INTAKE_ALGAE_L3,
              // SuperStructureState.SCORE_CORAL_L1,
              // SuperStructureState.SCORE_CORAL_L2,
              // SuperStructureState.SCORE_CORAL_L3,
              // SuperStructureState.SCORE_CORAL_L4,
              // SuperStructureState.SCORED_CORAL_L2,
              // SuperStructureState.SCORED_CORAL_L3,
              // SuperStructureState.SCORE_ALGAE_NET,
            },

            // SCORE_ALGAE_NET transitions
            new SuperStructureState[] {
              SuperStructureState.SCORE_ALGAE_NET,
              SuperStructureState.IDLE,
              SuperStructureState.IDLE_ALGAE,
              // SuperStructureState.INTAKE_CORAL,
              // SuperStructureState.INTAKE_ALGAE_L2,
              // SuperStructureState.INTAKE_ALGAE_L3,
              // SuperStructureState.SCORE_CORAL_L1,
              // SuperStructureState.SCORE_CORAL_L2,
              // SuperStructureState.SCORE_CORAL_L3,
              // SuperStructureState.SCORE_CORAL_L4,
              // SuperStructureState.SCORED_CORAL_L2,
              // SuperStructureState.SCORED_CORAL_L3,
              // SuperStructureState.SCORED_CORAL_L4,
            },
            // INTAKE_LOLLIPOP transitions
            new SuperStructureState[] {
              SuperStructureState.INTAKE_LOLLIPOP, SuperStructureState.IDLE_ALGAE,
              // SuperStructureState.INTAKE_CORAL,
              // SuperStructureState.INTAKE_ALGAE_L2,
              // SuperStructureState.INTAKE_ALGAE_L3,
              // SuperStructureState.SCORE_CORAL_L1,
              // SuperStructureState.SCORE_CORAL_L2,
              // SuperStructureState.SCORE_CORAL_L3,
              // SuperStructureState.SCORE_CORAL_L4,
              // SuperStructureState.SCORED_CORAL_L2,
              // SuperStructureState.SCORED_CORAL_L3,
              // SuperStructureState.SCORED_CORAL_L4,
            },

            // INTAKE_LOLLIPOP transitions
            new SuperStructureState[] {
              SuperStructureState.BEFORE_NET, SuperStructureState.SCORE_ALGAE_NET,
              // SuperStructureState.INTAKE_CORAL,
              // SuperStructureState.INTAKE_ALGAE_L2,
              // SuperStructureState.INTAKE_ALGAE_L3,
              // SuperStructureState.SCORE_CORAL_L1,
              // SuperStructureState.SCORE_CORAL_L2,
              // SuperStructureState.SCORE_CORAL_L3,
              // SuperStructureState.SCORE_CORAL_L4,
              // SuperStructureState.SCORED_CORAL_L2,
              // SuperStructureState.SCORED_CORAL_L3,
              // SuperStructureState.SCORED_CORAL_L4,
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
