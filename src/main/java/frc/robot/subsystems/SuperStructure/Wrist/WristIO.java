package frc.robot.subsystems.SuperStructure.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean wristConnected = false;
    public double positionRads = 0.0;
    // Real world is the position of the wrist taking the arm angle into account, with 0 being
    // horizontal
    public double realWorldPositionRads = 0.0;
    public double wristVelocityRadPerSec = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setVoltage(double outputVolts) {}
}
