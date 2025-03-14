package frc.robot.subsystems.SuperStructure;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.kArmStartingPositionRadians;

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
    public static final int kElevatorMotorCurrentLimit = 20;

    public static final double kElevatorMassKg = 12.0; // from CAD, elevator + arm + wrist
    public static final double kElevatorStartingPositionMeters = 0.0;
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = 0.78;
    public static final double kElevatorCarriageHeight = Units.inchesToMeters(36.0);
    // relative to robot origin
    public static final Translation2d kElevatorTranslation =
        new Translation2d(Units.inchesToMeters(9.5), 0.0);

    // In Meters/Sec
    public static final double kElevatorMaxVelocity = 1.5; // from sim, TBD
    // In Meters/SecSq
    public static final double kElevatorMaxAcceleration = 5.5; // from sim, TBD
    public static final double kElevatorMotorReduction = 35.0 * (22.0 / 18.0) * 1.25 * 1.72; // real
    public static final double kElevatorWheelRadiusMeters = 0.045; // real
    // Motor Rotations -> Elevator Meters
    public static final double kElevatorEncoderPositionFactor = 1.0 / 346.2;
    // Math.PI * 2 * kElevatorWheelRadiusMeters / kElevatorMotorReduction;
    // Motor RPM -> Elevator Meters/Sec
    public static final double kElevatorEncoderVelocityFactor =
        kElevatorEncoderPositionFactor / 60.0;
    // (Math.PI * 2 * kElevatorWheelRadiusMeters) / 60 / kElevatorMotorReduction;

    public static final double kElevatorKs = 0.11723;
    public static final double kElevatorKv = 43.594;
    public static final double kElevatorKa = 1.561;
    public static final double kElevatorKg = 0.024669;
    public static final double kElevatorKp = 30.0;
    public static final double kElevatorKd = 0.0;
    public static final double kElevatorLatencyCompensationMs = 0.025;

    public static final double kElevatorErrorToleranceMeters = 0.02;

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
    public static final double kArmStowedMOI_kgm2 = 0.442; // with gripper in
    public static final double kArmOutMOI_kgm2 = 1.404; // with gripper out
    public static final double kArmStartingPositionRadians = Units.degreesToRadians(84);
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
    public static final double kArmEncoderVelocityFactor = (2 * Math.PI) / 60 / kArmMotorReduction;

    public static final double kArmKs = 0.36613; // real
    public static final double kArmKv = 2.7196; // real
    public static final double kArmStowedKg = 0.1813; // real
    public static final double kArmOutKg = 0.65298; // real
    public static final double kArmKp = 8.0;
    public static final double kArmKi = 0.1;
    public static final double kArmKd = 0.1;

    public static final double kArmSimKv = 2.7196;
    public static final double kArmSimKg = 0.1813;
    public static final double kArmSimKp = 8.0;
    public static final double kArmSimKd = 0.1;

    public static final double kArmErrorToleranceRads = Units.degreesToRadians(5.0);

    @AllArgsConstructor
    public static enum ArmGoals {
      // Setpoints MUST be in radians when declared and degrees here! Please
      IDLE(TunableDouble.register("Arm/IDLE", 50.0)),
      STOW(() -> Units.radiansToDegrees(kArmStartingPositionRadians)),
      INTAKE(() -> 0.0),
      CORAL_STATION(TunableDouble.register("Arm/CORAL_STATION", 40.0)),
      REEF(TunableDouble.register("Arm/REEF", 45.0)),
      ALGAE_FLOOR(TunableDouble.register("Arm/ALGAE_FLOOR", -45.0));

      public DoubleSupplier setpointDegrees;
    }
  }

  public static class WristConstants {
    public static final int kWristSparkId = 34;
    public static final boolean kWristInverted = true;
    public static final int kWristMotorCurrentLimit = 20;

    public static final double kWristLengthMeters = Units.inchesToMeters(8.5);
    public static final double kWristMOI_kgm2 = 0.324;
    public static final double kWristCOGOffsetForFFRadians = -0.53379;
    // real world angle
    public static final double kWristStartingPositionRadians =
        kArmStartingPositionRadians + Units.degreesToRadians(180);
    // relative to arm
    public static final double kWristMinAngleRads = Units.degreesToRadians(30);
    public static final double kWristMaxAngleRads = Units.degreesToRadians(180);

    // In Rad/Sec
    public static final double kWristMaxVelocity = 4.5;
    // In Rad/SecSq
    public static final double kWristMaxAcceleration = 20;
    public static final double kWristMotorReduction = 116.67 / 1.7 * (7.0 / 4.0); // real
    // Motor Rotations -> Wrist Radians
    public static final double kWristEncoderPositionFactor = 2 * Math.PI / kWristMotorReduction;
    // Motor RPM -> Wrist Rad/Sec
    public static final double kWristEncoderVelocityFactor =
        (2 * Math.PI) / 60 / kWristMotorReduction;

    public static final double kWristKs = 0.19933; // real
    public static final double kWristKv = 1.1691; // real
    public static final double kWristKg = 0.64929; // real
    public static final double kWristKp = 8.0;
    public static final double kWristKi = 0.0;
    public static final double kWristKd = 0.5;

    public static final double kWristSimKv = 1.1691;
    public static final double kWristSimKg = 0.64929;
    public static final double kWristSimKp = 25.0;
    public static final double kWristSimKd = 0.5;

    public static final double kWristErrorToleranceRads = Units.degreesToRadians(3.0);

    @AllArgsConstructor
    public static enum WristGoals {
      // Setpoints MUST be in radians when declared and degrees here! Please
      // these are all in real-world measurements
      STOW(() -> Units.radiansToDegrees(kWristStartingPositionRadians)),
      INTAKE(() -> 180.0),
      REEF(TunableDouble.register("Wrist/REEF", 150.0)),
      L1REEF(TunableDouble.register("Wrist/L1REEF", 155.0)),
      L4REEF(TunableDouble.register("Wrist/L4REEF", 130.0)),
      ALGAE_FLOOR(TunableDouble.register("Wrist/ALGAE_FLOOR", 30.0)),
      ALGAE_REEF(TunableDouble.register("Wrist/ALGAE_REEF", 120.0)),
      PROCESSOR(TunableDouble.register("Wrist/PROCESSOR", 110.0)),
      NET(TunableDouble.register("Wrist/NET", 160.0)),
      LOCK(() -> -1.0);

      public DoubleSupplier setpointDegrees;
    }
  }
}
