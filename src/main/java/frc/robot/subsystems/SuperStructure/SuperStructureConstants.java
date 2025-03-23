package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.TunableDouble;
import java.util.function.DoubleSupplier;
import lombok.AllArgsConstructor;

public class SuperStructureConstants {
  public static class ElevatorConstants {
    public static final int kElevatorSparkId1 = 31;
    public static final int kElevatorSparkId2 = 30;
    public static final boolean kElevatorM1Inverted = true;
    public static final boolean kElevatorM2Opposite = true;
    public static final int kElevatorMotorCurrentLimit = 30;

    public static final double kElevatorStartingPositionMeters = 0.0;
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = 0.78;
    public static final double kElevatorCarriageHeight = Units.inchesToMeters(35.0);
    // relative to robot origin
    public static final Translation2d kElevatorTranslation =
        new Translation2d(Units.inchesToMeters(9.5), 0.0);

    // In Meters/Sec
    public static final double kElevatorMaxVelocity = 0.95;
    // In Meters/SecSq
    public static final double kElevatorMaxAcceleration = 3.0;
    // Motor Rotations -> Elevator Meters
    public static final double kElevatorEncoderPositionFactor = 1.0 / 86.0;
    // Motor RPM -> Elevator Meters/Sec
    public static final double kElevatorEncoderVelocityFactor =
        kElevatorEncoderPositionFactor / 60.0;

    public static final double kElevatorKs = 0.38922;
    public static final double kElevatorKv = 10.618;
    public static final double kElevatorKa = 0.53166;
    public static final double kElevatorKg = 0.5;
    public static final double kElevatorKp = 5.0;
    public static final double kElevatorKd = 0.0;
    public static final double kElevatorLatencyCompensationMs = 0.025;

    public static final double kElevatorErrorToleranceMeters = 0.01;

    @AllArgsConstructor
    public static enum ElevatorLevel {
      STOW(() -> kElevatorStartingPositionMeters),
      IDLE(TunableDouble.register("Elevator/IDLE", 0.1)),
      L2(TunableDouble.register("Elevator/L2", 0.15)),
      L4(TunableDouble.register("Elevator/L4", 0.65)),
      NET(TunableDouble.register("Elevator/NET", 0.7));

      public DoubleSupplier setpointMeters;
    }
  }

  public static class ArmConstants {
    public static final int kArmSparkId = 43;
    public static final boolean kArmInverted = false;
    public static final int kArmMotorCurrentLimit = 30;

    public static final double kArmLengthMeters = 0.58;
    public static final double kArmStartingPositionRads = Units.degreesToRadians(-90);
    public static final double kArmMinAngleRads = Units.degreesToRadians(-90);
    public static final double kArmMaxAngleRads = Units.degreesToRadians(120);

    // In Rad/Sec
    public static final double kArmMaxVelocity = 7.0;
    // In Rad/SecSq
    public static final double kArmMaxAcceleration = 35.0;
    // Motor Rotations -> Arm Radians
    public static final double kArmEncoderPositionFactor = 1.0 / 13.35;
    public static final double kArmMotorReduction = 2 * Math.PI / kArmEncoderPositionFactor; // real
    // Motor RPM -> Arm Rad/Sec
    public static final double kArmEncoderVelocityFactor = kArmEncoderPositionFactor / 60;
    public static final double kArmKs = 0.4004;
    public static final double kArmKv = 1.276;
    public static final double kArmKa = 0.069168;
    public static final double kArmKg = 0.2563;
    public static final double kArmKp = 5.0;
    public static final double kArmKd = 0.0;
    public static final double kArmLatencyCompensationMs = 0.025;

    public static final double kArmErrorToleranceRads = Units.degreesToRadians(5.0);

    @AllArgsConstructor
    public static enum ArmGoals {
      // Setpoints MUST be in radians when declared and degrees here! Please
      // STOW(() -> Units.radiansToDegrees(kArmStartingPositionRads)),
      // IDLE_CORAL(TunableDouble.register("Arm/IDLE_CORAL", -80.0)),
      IDLE(() -> Units.radiansToDegrees(kArmStartingPositionRads)),
      IDLE_ALGAE(TunableDouble.register("Arm/IDLE_ALGAE", -60.0)),
      ALGAE_L2(TunableDouble.register("Arm/ALGAE_L2", -10.0)),
      ALGAE_L3(TunableDouble.register("Arm/ALGAE_L3", 30.0)),
      CORAL_L2(TunableDouble.register("Arm/CORAL_L2", -65.0)),
      // TODO
      SCORED_CORAL_L2(TunableDouble.register("Arm/SCORED_CORAL_L2", -20.0)),
      CORAL_REEF_HIGH(TunableDouble.register("Arm/CORAL_REEF_HIGH", 40.0)),
      // TODO
      SCORED_REEF_HIGH(TunableDouble.register("Arm/SCORED_REEF_HIGH", -20.0)),
      NET(TunableDouble.register("Arm/NET", 65.0));

      public DoubleSupplier setpointDegrees;
    }
  }
}
