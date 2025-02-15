package frc.robot.io.beambreak;

import org.littletonrobotics.junction.AutoLog;

/** Base interface for a beam break sensor. */
public interface BeamBreakIO {
  @AutoLog
  public static class BeamBreakIOInputs {
    public boolean state = false;
  }

  public default void updateInputs(BeamBreakIOInputs inputs) {}
}
