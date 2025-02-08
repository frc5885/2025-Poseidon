package frc.robot.subsystems.EndAffecter.AlgaeClaw;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeClawIO {
  @AutoLog
  public static class AlgaeClawIOIntputs {
    public double positionRotations = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean AlfeaClawConnected = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AlgaeClawIOIntputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}
}
