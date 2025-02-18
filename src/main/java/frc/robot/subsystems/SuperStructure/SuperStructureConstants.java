package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableDouble;
import java.util.function.DoubleSupplier;
import lombok.AllArgsConstructor;

public class SuperStructureConstants {
  public static class ElevatorConstants {
    public static final int kElevatorSparkId1 = 40;
    public static final int kElevatorSparkId2 = 41;
    public static final boolean kElevatorM1Inverted = false;
    public static final boolean kElevatorM2Inverted = !kElevatorM1Inverted;
    public static final int kElevatorMotorCurrentLimit = 40;

    public static final double kElevatorMassKg = 4.0;
    public static final double kElevatorStartingPositionMeters = 0.0;
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = Units.inchesToMeters(56.625);
    public static final double kElevatorCarriageHeight = Units.inchesToMeters(7.375);
    public static final double kElevatorStage1MaxTravel = Units.inchesToMeters(30.0);
    // relative to robot origin
    public static final Translation2d kElevatorTranslation =
        new Translation2d(Units.inchesToMeters(9.5), 0.0);

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

    public static final double kElevatorKs = 0.0;
    public static final double kElevatorKv = 0.0;
    public static final double kElevatorKg = 0.0;
    public static final double kElevatorKp = 0.0;
    public static final double kElevatorKd = 0.0;

    public static final double kElevatorSimKv = 3.9;
    public static final double kElevatorSimKg = 0.46;
    public static final double kElevatorSimKp = 0.0;
    public static final double kElevatorSimKd = 0.0;

    public static final double kElevatorErrorToleranceMeters = 0.005;

    @AllArgsConstructor
    public static enum ElevatorLevel {
      STOW(() -> kElevatorMinHeightMeters),
      L1(TunableDouble.register("Elevator/L1", 0.1)),
      ALGAE_L2(TunableDouble.register("Elevator/L2ALGAE", 0.2)),
      L2(TunableDouble.register("Elevator/L2", 0.4)),
      ALGAE_L3(TunableDouble.register("Elevator/L3ALGAE", 0.57)),
      L3(TunableDouble.register("Elevator/L3", 0.77)),
      L4(() -> kElevatorMaxHeightMeters);

      public DoubleSupplier setpointMeters;
    }
  }

  public static class ArmConstants {
    public static final int kArmSparkId = 42;
    public static final boolean kArmInverted = false;
    public static final int kArmMotorCurrentLimit = 40;

    public static final double kArmLengthMeters = Units.inchesToMeters(14.0);
    public static final double kArmStartingPositionRadians = Units.degreesToRadians(90);
    public static final double kArmMinAngleRads = Units.degreesToRadians(-45);
    public static final double kArmMaxAngleRads = Units.degreesToRadians(90.0);

    // In Rad/Sec
    public static final double kArmMaxVelocity = Units.degreesToRadians(2500.0);
    // In Rad/SecSq
    public static final double kArmMaxAcceleration = Units.degreesToRadians(5000.0);
    public static final double kArmMotorReduction = 240.0;
    // Motor Rotations -> Arm Radians
    public static final double kArmEncoderPositionFactor = 2 * Math.PI / kArmMotorReduction;
    // Motor RPM -> Arm Rad/Sec
    public static final double kArmEncoderVelocityFactor = (2 * Math.PI) / 60 / kArmMotorReduction;

    public static final double kArmKs = 0.0;
    public static final double kArmKv = 0.0;
    public static final double kArmKg = 0.0;
    public static final double kArmKp = 0.0;
    public static final double kArmKd = 0.0;

    public static final double kArmSimKv = 4.7706;
    public static final double kArmSimKg = 2.6085;
    public static final double kArmSimKp = 1.0;
    public static final double kArmSimKd = 0.0;

    public static final double kArmErrorToleranceRads = Units.degreesToRadians(1.5);

    @AllArgsConstructor
    public static enum ArmGoals {
      STOW(() -> 90.0),
      INTAKE(() -> 0.0),
      REEF(TunableDouble.register("Arm/REEF", 45.0)),
      ALGAE_FLOOR(TunableDouble.register("Arm/ALGAE_FLOOR", -45.0));

      public DoubleSupplier setpointDegrees;
    }
  }

  public static class WristConstants {
    public static final int kWristSparkId = 43;
    public static final boolean kWristInverted = false;
    public static final int kWristMotorCurrentLimit = 40;

    public static final double kWristLengthMeters = Units.inchesToMeters(8.5);
    // real world angle
    public static final double kWristStartingPositionRadians = Units.degreesToRadians(270);
    // relative to arm
    public static final double kWristMinAngleRads = Units.degreesToRadians(30);
    public static final double kWristMaxAngleRads = Units.degreesToRadians(180);

    // In Rad/Sec
    public static final double kWristMaxVelocity = Units.degreesToRadians(2500.0);
    // In Rad/SecSq
    public static final double kWristMaxAcceleration = Units.degreesToRadians(5000.0);
    public static final double kWristMotorReduction = 50.0;
    // Motor Rotations -> Wrist Radians
    public static final double kWristEncoderPositionFactor = 2 * Math.PI / kWristMotorReduction;
    // Motor RPM -> Wrist Rad/Sec
    public static final double kWristEncoderVelocityFactor =
        (2 * Math.PI) / 60 / kWristMotorReduction;

    public static final double kWristKs = 0.0;
    public static final double kWristKv = 0.0;
    public static final double kWristKg = 0.0;
    public static final double kWristKp = 0.0;
    public static final double kWristKd = 0.0;

    public static final double kWristSimKv = 0.85;
    public static final double kWristSimKg = 5.0;
    public static final double kWristSimKp = 1.0;
    public static final double kWristSimKd = 2.0;

    public static final double kWristErrorToleranceRads = Units.degreesToRadians(1.5);

    @AllArgsConstructor
    public static enum WristGoals {
      // these are all in real-world radians
      STOW(() -> 270.0),
      INTAKE(() -> 180.0),
      REEF(TunableDouble.register("Wrist/REEF", 150.0)),
      L1REEF(TunableDouble.register("Wrist/L1REEF", 155.0)),
      L4REEF(TunableDouble.register("Wrist/L4REEF", 130.0)),
      ALGAE_FLOOR(TunableDouble.register("Wrist/ALGAE_FLOOR", 30.0)),
      ALGAE_REEF(TunableDouble.register("Wrist/ALGAE_REEF", 120.0)),
      PROCESSOR(TunableDouble.register("Wrist/PROCESSOR", 120.0)),
      NET(TunableDouble.register("Wrist/NET", 160.0)),
      LOCK(() -> -1.0);

      public DoubleSupplier setpointDegrees;
    }
  }
}
