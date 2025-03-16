package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableDouble;
import java.util.function.DoubleSupplier;
import lombok.AllArgsConstructor;

public class SuperStructureConstants {
  public static class ElevatorConstants {
    public static final int kElevatorSparkId1 = 30;
    public static final int kElevatorSparkId2 = 31;
    public static final boolean kElevatorM1Inverted = false;
    public static final boolean kElevatorM2Opposite = true;
    public static final int kElevatorMotorCurrentLimit = 30;

    public static final double kElevatorStartingPositionMeters = 0.0;
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = 0.78;
    public static final double kElevatorCarriageHeight = Units.inchesToMeters(36.0);
    // relative to robot origin
    public static final Translation2d kElevatorTranslation =
        new Translation2d(Units.inchesToMeters(9.5), 0.0);

    // In Meters/Sec
    public static final double kElevatorMaxVelocity = 1.0;
    // In Meters/SecSq
    public static final double kElevatorMaxAcceleration = 5.0;
    // Motor Rotations -> Elevator Meters
    public static final double kElevatorEncoderPositionFactor = 1.0 / 86.0;
    // Motor RPM -> Elevator Meters/Sec
    public static final double kElevatorEncoderVelocityFactor =
        kElevatorEncoderPositionFactor / 60.0;

    public static final double kElevatorKs = 0.29236;
    public static final double kElevatorKv = 10.292;
    public static final double kElevatorKa = 0.68893;
    public static final double kElevatorKg = 0.065582;
    public static final double kElevatorKp = 5.0;
    public static final double kElevatorKd = 0.0;
    public static final double kElevatorLatencyCompensationMs = 0.025;

    public static final double kElevatorErrorToleranceMeters = 0.01;

    @AllArgsConstructor
    public static enum ElevatorLevel {
      STOW(() -> kElevatorMinHeightMeters),
      CORAL_STATION_TRANSITION(TunableDouble.register("Elevator/CORAL_STATION_TRANSITION", 0.4)),
      CORAL_STATION(TunableDouble.register("Elevator/CORAL_STATION", 0.5)),
      L1(TunableDouble.register("Elevator/L1", 0.1)),
      ALGAE_L2(TunableDouble.register("Elevator/L2ALGAE", 0.1)),
      L2(TunableDouble.register("Elevator/L2", 0.4)),
      ALGAE_L3(TunableDouble.register("Elevator/L3ALGAE", 0.57)),
      L3(TunableDouble.register("Elevator/L3", 0.77)),
      L4(() -> kElevatorMaxHeightMeters),
      TEST(TunableDouble.register("Elevator/TEST", STOW.setpointMeters.getAsDouble()));

      public DoubleSupplier setpointMeters;
    }
  }

  public static class ArmConstants {
    public static final int kArmSparkId = 43;
    public static final boolean kArmInverted = true;
    public static final int kArmMotorCurrentLimit = 20;

    public static final double kArmLengthMeters = Units.inchesToMeters(14.0);
    public static final double kArmStartingPositionRads = Units.degreesToRadians(84);
    public static final double kArmMinAngleRads = Units.degreesToRadians(-45);
    public static final double kArmMaxAngleRads = Units.degreesToRadians(90.0);

    // In Rad/Sec
    public static final double kArmMaxVelocity = 5.0; // from sim, TBD
    // In Rad/SecSq
    public static final double kArmMaxAcceleration = 30.0; // from sim, TBD
    public static final double kArmMotorReduction = 190.55 * 1.4; // real
    // Motor Rotations -> Arm Radians
    public static final double kArmEncoderPositionFactor = 2 * Math.PI / kArmMotorReduction;
    // Motor RPM -> Arm Rad/Sec
    public static final double kArmEncoderVelocityFactor = kArmEncoderPositionFactor / 60;

    public static final double kArmKs = 0.36613; // real
    public static final double kArmKv = 2.7196; // real
    public static final double kArmKa = 0.01;
    public static final double kArmKg = 0.01;
    public static final double kArmKp = 8.0;
    public static final double kArmKd = 0.1;
    public static final double kArmLatencyCompensationMs = 0.025;

    public static final double kArmErrorToleranceRads = Units.degreesToRadians(5.0);

    @AllArgsConstructor
    public static enum ArmGoals {
      // Setpoints MUST be in radians when declared and degrees here! Please
      IDLE(TunableDouble.register("Arm/IDLE", 50.0)),
      STOW(() -> Units.radiansToDegrees(kArmStartingPositionRads)),
      INTAKE(() -> 0.0),
      CORAL_STATION(TunableDouble.register("Arm/CORAL_STATION", 40.0)),
      REEF(TunableDouble.register("Arm/REEF", 45.0)),
      ALGAE_FLOOR(TunableDouble.register("Arm/ALGAE_FLOOR", -45.0));

      public DoubleSupplier setpointDegrees;
    }
  }
}
