package frc.robot.subsystems.Feeder;

public class FeederConstants {

  public static enum FeederState {
    FEEDING,
    FEEDING_SLOW,
    IDLE
  }

  public static final int kMotorId1 = 37;
  public static final int kMotorId2 = 38;
  public static final boolean kMotor1Inverted = false;
  public static final boolean kMotor2Opposite = true;
  public static final int kCurrentLimit = 20;
  public static final double kGearRatio = 1.0;
  public static final double kFeederCurrentThreshold = 15;
  public static final int kBeamBreakId = 0;
}
