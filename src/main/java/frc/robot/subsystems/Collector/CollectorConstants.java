package frc.robot.subsystems.Collector;

public class CollectorConstants {
  public static class FeederConstants {
    public static final int kMotorId1 = 37;
    public static final int kMotorId2 = 38;
    public static final boolean kMotor1Inverted = false;
    public static final boolean kMotor2Opposite = true;
    public static final int kCurrentLimit = 20;
    public static final double kGearRatio = 1.0;
  }

  public static class IntakeConstants {
    public static final int kMotorId1 = 35;
    public static final int kMotorId2 = 39;
    public static final int kBeamBreakId = 6;

    public static final boolean kMotor1Inverted = false;
    public static final boolean kMotor2Opposite = false;
    public static final int kCurrentLimit = 20;
    public static final double kGearRatio = 1.0;

    public static final int kSolenoidId1 = 0;
    public static final int kSolenoidId2 = 1;
    public static final int kPneumaticHubCanID = 35;
  }
}
