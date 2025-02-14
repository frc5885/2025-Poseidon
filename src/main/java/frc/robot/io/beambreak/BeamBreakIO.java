package frc.robot.io.beambreak;

/** Base interface for a beam break sensor. */
public interface BeamBreakIO {
  /**
   * Returns whether the sensor is currently triggered.
   *
   * @return true if a game piece is detected, false otherwise.
   */
  boolean getIsTriggered();
}
