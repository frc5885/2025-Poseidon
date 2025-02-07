package frc.robot.subsystems.Collector;

public class CollectorConstants {
    public static class IntakeConstants {
        public static final int intakeId1 = 7;
        public static final int intakeId2 = 8;
        public static final boolean intakeMotor1Inverted = false;
        public static final boolean intakeMotor2Inverted = !intakeMotor1Inverted;

        public static final int intakeMotorCurrentLimit = 40;

        public static final double intakeGearRatio = 1.0;
    }
}
