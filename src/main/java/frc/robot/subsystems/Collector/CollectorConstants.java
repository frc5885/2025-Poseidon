package frc.robot.subsystems.Collector;

public class CollectorConstants {
  public static class FeederConstants {
    public static final int kMotorId = 9;
    public static final boolean kInverted = false;
    public static final int kCurrentLimit = 40;
    public static final double kGearRatio = 1.0;
  }

  public static class IntakeConstants {
    public static final int kMotorId1 = 7;
    public static final int kMotorId2 = 8;
    public static final boolean kMotor1Inverted = false;
    public static final boolean kMotor2Inverted = !kMotor1Inverted;
    public static final int kCurrentLimit = 40;
    public static final double kGearRatio = 1.0;
  }
}
