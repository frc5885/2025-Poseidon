package frc.robot.subsystems.SuperStructure.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean armConnected = false;
    public double absolutePositionRads = 0.0;
    public double positionRads = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrentAmps = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(double outputVolts) {}
}
