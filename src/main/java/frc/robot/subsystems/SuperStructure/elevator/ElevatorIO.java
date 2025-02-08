package frc.robot.subsystems.SuperStructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean motor1Connected = false;
    public boolean motor2Connected = false;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = {0.0, 0.0};
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setVoltage(double outputVolts) {}
}
