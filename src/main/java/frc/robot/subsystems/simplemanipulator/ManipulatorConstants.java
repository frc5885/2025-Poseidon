package frc.robot.subsystems.simplemanipulator;

import edu.wpi.first.math.util.Units;

public class ManipulatorConstants {
  // TODO temporary
  public static class ElevatorConstants {
    public static final int kElevatorSparkId = 40;
    public static final boolean kElevatorInverted = false;
    public static final int kElevatorMotorCurrentLimit = 40;

    public static final double kElevatorLowerBoundMeters = -20.0;
    public static final double kElevatorUpperBoundMeters = 20.0;
    // In Meters/Sec
    public static final double kElevatorMaxVelocity = 1.75;
    // In Meters/SecSq
    public static final double kElevatorMaxAcceleration = 0.75;
    public static final double kElevatorMotorReduction = 10.0;
    public static final double kElevatorWheelRadiusMeters = Units.inchesToMeters(2.0);
    // Motor Rotations -> Elevator Meters
    public static final double kElevatorEncoderPositionFactor =
        Math.PI * 2 * kElevatorWheelRadiusMeters / kElevatorMotorReduction;
    // Motor RPM -> Elevator Meters/Sec
    public static final double kElevatorEncoderVelocityFactor =
        (Math.PI * 2 * kElevatorWheelRadiusMeters) / 60 / kElevatorMotorReduction;

    public static final double elevatorKs = 0.0;
    public static final double elevatorKv = 0.0;
    public static final double elevatorKg = 0.0;
    public static final double elevatorKp = 0.0;
    public static final double elevatorKd = 0.0;
    public static final double kElevatorErrorToleranceMeters = 0.01;

    public static final double elevatorSimKp = 5.0;
    public static final double elevatorSimKd = 0.0;
    public static final double elevatorSimKv = 0.1;
    public static final double elevatorSimKg = 0.98;
  }
}
