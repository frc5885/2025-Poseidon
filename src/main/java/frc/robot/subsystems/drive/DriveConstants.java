// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class DriveConstants {
  public static final double kOdometryFrequency = 100.0; // Hz
  public static final double kTrackWidth = Units.inchesToMeters(24.25);
  public static final double kWheelBase = Units.inchesToMeters(24.25);
  public static final double kDriveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
  public static final Translation2d[] kModuleTranslations =
      new Translation2d[] {
        new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
        new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
        new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
        new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d kFrontLeftZeroRotation = new Rotation2d(5.844648404296888);
  public static final Rotation2d kFrontRightZeroRotation = new Rotation2d(0.617795977602757);
  public static final Rotation2d kBackLeftZeroRotation = new Rotation2d(3.0154699868813046);
  public static final Rotation2d kBackRightZeroRotation = new Rotation2d(6.260559713965955);

  // Device CAN IDs
  public static final int kFrontLeftDriveCanId = 13;
  public static final int kBackLeftDriveCanId = 10;
  public static final int kFrontRightDriveCanId = 12;
  public static final int kBackRightDriveCanId = 11;

  public static final int kFrontLeftTurnCanId = 23;
  public static final int kBackLeftTurnCanId = 20;
  public static final int kFrontRightTurnCanId = 22;
  public static final int kBackRightTurnCanId = 21;

  // Absolute encoder analog ports
  public static final int kFrontLeftAbsoluteEncoderPort = 3;
  public static final int kBackLeftAbsoluteEncoderPort = 0;
  public static final int kFrontRightAbsoluteEncoderPort = 2;
  public static final int kBackRightAbsoluteEncoderPort = 1;

  // Drive motor configuration
  public static final int kDriveMotorCurrentLimit = 35;
  public static final double kWheelRadiusMeters = Units.inchesToMeters(1.944);
  public static final double kDriveMotorReduction = 6.75 / 1.0; // SDS Mk4i L2
  public static final DCMotor kDriveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double kDriveEncoderPositionFactor =
      2 * Math.PI / kDriveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double kDriveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / kDriveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Turn motor configuration
  public static final boolean kTurnInverted = true;
  public static final int kTurnMotorCurrentLimit = 25;
  public static final double kTurnMotorReduction = (150.0 / 7.0) / 1.0;
  public static final DCMotor kTurnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean kTurnEncoderInverted = false;
  public static final double kTurnEncoderPositionFactor =
      2 * Math.PI / kTurnMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double kTurnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / kTurnMotorReduction; // Rotor RPM -> Wheel Rad/Sec
  public static final double kTurnAbsoluteEncoderPositionFactor =
      2 * Math.PI; // Rotations -> Radians
  public static final double kTurnAbsoluteEncoderVelocityFactor =
      (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Drive PID configuration
  public static final double kDriveKp = 0.0;
  public static final double kDriveKd = 0.0;
  public static final double kDriveKs = 0.039644;
  public static final double kDriveKv = 0.13951;
  public static final double kDriveKa = 0.0052922;
  public static final double kDriveSimP = 0.05;
  public static final double kDriveSimD = 0.0;
  public static final double kDriveSimKs = 0.047123;
  public static final double kDriveSimKv = 0.15668;
  public static final double kDriveSimKa = 0.013224;

  // Turn PID configuration
  public static final double kTurnKp = 5.5;
  public static final double kTurnKd = 0.0;
  public static final double kTurnSimP = 8.0;
  public static final double kTurnSimD = 0.0;
  public static final double kTurnKs = 0.15515;
  public static final double kTurnKv = 0.42405;
  // public static final double kTurnKa = 0.010624;
  public static final double kTurnKa = 0.008291;

  public static final double kTurnSimKv = 0.42421;
  public static final double kTurnSimKa = 0.008291;
  public static final double kTurnPIDMinInput = 0; // Radians
  public static final double kTurnPIDMaxInput = 2 * Math.PI; // Radians
  // these seem to work much better multiplied by ~10, no idea why
  // public static final double kTurnMaxVelocityRadPerSec =
  //     Units.radiansPerSecondToRotationsPerMinute(28.0); // measured as 28.0
  // public static final double kTurnMaxAccelerationRadPerSecSq =
  //     Units.radiansPerSecondToRotationsPerMinute(28.0 / 0.17); // measured as 28.0/0.17
  public static final double kTurnMaxErrorTolerance = Units.degreesToRadians(2.0);

  // Speed and acceleration constraints
  public static final double kMaxSpeedMetersPerSec = 3.6; // measured in sim
  public static final double kMaxAccelerationMetersPerSecSq = 5.4; // measured in sim
  public static final double kMaxAngularSpeedRadiansPerSec =
      kMaxSpeedMetersPerSec / kDriveBaseRadius;
  public static final double kMaxAngularAccelerationRadiansPerSecSq = 16; // measured in sim
  public static final double kMaxModuleRotationVelocityRadiansPerSec = 15; // measured in sim

  // PathPlanner configuration
  public static final double kRobotMassKg = 36; // TODO: measure
  public static final double kRobotMOI = 3.0; // can be calculated from sysID, check PP docs
  public static final double kWheelCOF = 0.899; // maple sim constant for colsons
  public static final double kDistanceTolerance = 0.02; // meters
  public static final double kRotationTolerance = 4.0; // degrees
  public static final RobotConfig kPPConfig =
      new RobotConfig(
          kRobotMassKg,
          kRobotMOI,
          new ModuleConfig(
              kWheelRadiusMeters,
              kMaxSpeedMetersPerSec,
              kWheelCOF,
              kDriveGearbox.withReduction(kDriveMotorReduction),
              kDriveMotorCurrentLimit,
              1),
          kModuleTranslations);

  public static final PPHolonomicDriveController kPPController =
      new PPHolonomicDriveController(new PIDConstants(3.0, 0.0), new PIDConstants(3.0, 0.0));

  public static final PathConstraints kPathConstraintsFast =
      new PathConstraints(
          kMaxSpeedMetersPerSec,
          kMaxAccelerationMetersPerSecSq,
          kMaxAngularSpeedRadiansPerSec,
          kMaxAngularAccelerationRadiansPerSecSq,
          12.0);

  public static final PathConstraints kPathConstraintsSlow =
      new PathConstraints(
          kMaxSpeedMetersPerSec * 0.5,
          kMaxAccelerationMetersPerSecSq * 0.5,
          kMaxAngularSpeedRadiansPerSec * 0.5,
          kMaxAngularAccelerationRadiansPerSecSq * 0.5,
          12.0);

  public static final DriveTrainSimulationConfig kMapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withCustomModuleTranslations(kModuleTranslations)
          .withRobotMass(Kilogram.of(kRobotMassKg))
          .withGyro(COTS.ofNav2X())
          .withSwerveModule(COTS.ofMark4i(kDriveGearbox, kTurnGearbox, COTS.WHEELS.COLSONS.cof, 2))
          .withBumperSize(Inches.of(37.5), Inches.of(37.5));
}
