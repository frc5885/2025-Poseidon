package frc.robot.subsystems.simplemanipulator.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean elevatorM1Connected = false;
    public boolean elevatorM2Connected = false;
    public double elevatorPositionMeters = 0.0;
    public double elevatorVelocityMetersPerSec = 0.0;
    public double elevatorAppliedVolts = 0.0;
    public double[] elevatorCurrentAmps = {0.0, 0.0};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double outputVolts) {}
}
