package frc.robot.subsystems.Collector;

public class CollectorConstants {
  public static class FeederConstants {
    public static final int feederId = 9;
    public static final boolean feederMotorInverted = false;
    public static final int feederMotorCurrentLimit = 40;
    public static final double feederGearRatio = 1.0;
  }

  public static class IntakeConstants {
    public static final int intakeId1 = 7;
    public static final int intakeId2 = 8;
    public static final boolean intakeMotor1Inverted = false;
    public static final boolean intakeMotor2Inverted = !intakeMotor1Inverted;

    public static final int intakeMotorCurrentLimit = 40;

    public static final double intakeGearRatio = 1.0;
  }

  public static class CoralEjecterConstants {
    public static final int CoralEjecterId = 11;
    public static final boolean CoralEjecterInverted = false;
    public static final int CoralEjecterCurrentLimit = 40;
    public static final double CoralEjecterGearRatio = 1.0;
  }

  public static class AlgaeClawConstants {
    public static final int AlgaeClawId = 11;
    public static final boolean AlgaeClawInverted = false;
    public static final int AlgaeClawCurrentLimit = 40;
    public static final double AlgaeClawGearRatio = 1.0;
  }
}
