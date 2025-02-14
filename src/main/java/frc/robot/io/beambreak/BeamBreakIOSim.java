package frc.robot.io.beambreak;

import edu.wpi.first.wpilibj.Timer;

/**
 * Simulated beam break sensor for testing purposes. The sensor will report a change in state after
 * a delay. Call simulateGamePieceAcquisition() to simulate a game piece being acquired. Call
 * simulateGamePieceRemoval() to simulate a game piece being removed.
 */
public class BeamBreakIOSim implements BeamBreakIO {

  // The current state reported by the sensor.
  private boolean sensorState = false;

  // The target state we want the sensor to eventually report.
  private boolean targetState = false;

  // Time when the last state change was initiated.
  private double stateChangeStartTime = -1;

  // Delay in seconds before the sensor state actually updates.
  private static final double DELAY_SECONDS = 1.0;

  /**
   * Simulate a game piece being acquired. The sensor will report false for DELAY_SECONDS before
   * switching to true.
   */
  public void simulateGamePieceAcquisition() {
    if (!targetState) { // Only change if not already targeting acquisition.
      targetState = true;
      stateChangeStartTime = Timer.getFPGATimestamp();
    }
  }

  /**
   * Simulate a game piece being removed. The sensor will report true for DELAY_SECONDS before
   * switching to false.
   */
  public void simulateGamePieceRemoval() {
    if (targetState) { // Only change if not already targeting removal.
      targetState = false;
      stateChangeStartTime = Timer.getFPGATimestamp();
    }
  }

  /**
   * Returns the current sensor state. Internally, if a state change was initiated and the delay has
   * passed, the sensor state is updated to the target state.
   */
  @Override
  public boolean getIsTriggered() {
    double currentTime = Timer.getFPGATimestamp();

    // If there's a pending state change and the delay has passed, update sensor state.
    if (sensorState != targetState && stateChangeStartTime > 0) {
      if (currentTime - stateChangeStartTime >= DELAY_SECONDS) {
        sensorState = targetState;
        stateChangeStartTime = -1; // Reset the timer.
      }
    }

    return sensorState;
  }
}
