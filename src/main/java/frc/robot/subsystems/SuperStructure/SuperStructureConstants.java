package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.util.Units;

public class SuperStructureConstants {
  // TODO temporary
  public static class ElevatorConstants {
    public static final int kElevatorSparkId1 = 40;
    public static final int kElevatorSparkId2 = 41;
    public static final boolean kElevatorM1Inverted = false;
    public static final boolean kElevatorM2Inverted = !kElevatorM1Inverted;
    public static final int kElevatorMotorCurrentLimit = 40;

    public static final double kElevatorMassKg = 4.0;
    public static final double kElevatorStartingPositionMeters = 0.0;
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = 2.0;

    // In Meters/Sec
    public static final double kElevatorMaxVelocity = 1.54;
    // In Meters/SecSq
    public static final double kElevatorMaxAcceleration = 29;
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

    public static final double elevatorSimKv = 3.9;
    public static final double elevatorSimKg = 0.46;
    public static final double elevatorSimKp = 0.0;
    public static final double elevatorSimKd = 0.0;

    public static final double kElevatorErrorToleranceMeters = 0.005;

    public static enum ElevatorLevel {
      L1(0.0),
      L2(0.5),
      L3(1.0),
      L4(1.5);

      public double setpointMeters;

      private ElevatorLevel(double setpointMeters) {
        this.setpointMeters = setpointMeters;
      }
    }
  }

  public static class ArmConstants {
    public static final int kArmSparkId = 42;
    public static final boolean kArmInverted = false;
    public static final int kArmMotorCurrentLimit = 40;

    public static final double kArmLengthMeters = 0.3;
    public static final double kArmStartingPositionRadians = Units.degreesToRadians(-60.0);
    public static final double kArmMinAngleRads = Units.degreesToRadians(-60.0);
    public static final double kArmMaxAngleRads = Units.degreesToRadians(90.0);

    // In Rad/Sec
    public static final double kArmMaxVelocity = Units.degreesToRadians(2500.0);
    // In Rad/SecSq
    public static final double kArmMaxAcceleration = Units.degreesToRadians(5000.0);
    public static final double kArmMotorReduction = 30.0;
    // Motor Rotations -> Arm Radians
    public static final double kArmEncoderPositionFactor = 2 * Math.PI / kArmMotorReduction;
    // Motor RPM -> Arm Rad/Sec
    public static final double kArmEncoderVelocityFactor = (2 * Math.PI) / 60 / kArmMotorReduction;

    public static final double armKs = 0.0;
    public static final double armKv = 0.0;
    public static final double armKg = 0.0;
    public static final double armKp = 0.0;
    public static final double armKd = 0.0;

    public static final double armSimKv = 0.14;
    public static final double armSimKg = 8.895;
    public static final double armSimKp = 18.0;
    public static final double armSimKd = 9.0;

    public static final double kArmErrorToleranceRads = Units.degreesToRadians(1.5);

    public static enum ArmGoals {
      STOW(kArmMinAngleRads),
      RAISED(kArmMaxAngleRads),
      SETPOINT(Units.degreesToRadians(45.0));

      public double setpointRadians;

      private ArmGoals(double setpointRads) {
        setpointRadians = setpointRads;
      }
    }
  }
}
