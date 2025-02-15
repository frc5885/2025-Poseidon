package frc.robot.io.beambreak;

import edu.wpi.first.wpilibj.Timer;

/**
 * Simulated beam break sensor for testing purposes. The sensor will report a change in state after
 * a delay. Call simulateGamePieceAcquisition() to simulate a game piece being acquired. Call
 * simulateGamePieceRemoval() to simulate a game piece being removed.
 */
public class BeamBreakIOSim implements BeamBreakIO {

  // The current state reported by the sensor.
  private boolean m_sensorState = false;

  // The target state we want the sensor to eventually report.
  private boolean m_targetState = false;

  // Time when the last state change was initiated.
  private double m_stateChangeStartTime = -1;

  // Delay before the sensor state changes.
  private double m_delaySeconds = 1.0;

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    double currentTime = Timer.getFPGATimestamp();

    // If there's a pending state change and the delay has passed, update sensor state.
    if (m_sensorState != m_targetState && m_stateChangeStartTime > 0) {
      if (currentTime - m_stateChangeStartTime >= m_delaySeconds) {
        m_sensorState = m_targetState;
        m_stateChangeStartTime = -1; // Reset the timer.
      }
    }

    inputs.state = m_sensorState;
  }

  /**
   * Simulate a game piece being acquired. The sensor will report false for delaySeconds before
   * switching to true.
   */
  public void simulateGamePieceIntake(double delaySeconds) {
    m_delaySeconds = delaySeconds;
    if (!m_targetState) { // Only change if not already targeting acquisition.
      m_targetState = true;
      m_stateChangeStartTime = Timer.getFPGATimestamp();
    }
  }

  /**
   * Simulate a game piece being removed. The sensor will report true for delaySeconds before
   * switching to false.
   */
  public void simulateGamePieceOuttake(double delaySeconds) {
    m_delaySeconds = delaySeconds;
    if (m_targetState) { // Only change if not already targeting removal.
      m_targetState = false;
      m_stateChangeStartTime = Timer.getFPGATimestamp();
    }
  }

  /**
   * Cancel a simulated game piece change. The sensor will report the current state immediately and
   * will not change.
   */
  public void cancelSimulatedGamePieceChange() {
    m_targetState = m_sensorState;
    m_stateChangeStartTime = -1;
  }
}
