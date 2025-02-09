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

import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.TunablePIDController;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
  private final Rotation2d zeroRotation;

  // Hardware objects
  private final SparkMax driveSpark;
  private final SparkMax turnSpark;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnEncoder;
  private final AnalogInput turnAbsoluteEncoder;

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;
  private final TunablePIDController turnPID;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIOSpark(int module) {
    zeroRotation =
        switch (module) {
          case 0 -> kFrontLeftZeroRotation;
          case 1 -> kFrontRightZeroRotation;
          case 2 -> kBackLeftZeroRotation;
          case 3 -> kBackRightZeroRotation;
          default -> new Rotation2d();
        };
    driveSpark =
        new SparkMax(
            switch (module) {
              case 0 -> kFrontLeftDriveCanId;
              case 1 -> kFrontRightDriveCanId;
              case 2 -> kBackLeftDriveCanId;
              case 3 -> kBackRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> kFrontLeftTurnCanId;
              case 1 -> kFrontRightTurnCanId;
              case 2 -> kBackLeftTurnCanId;
              case 3 -> kBackRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getEncoder();
    turnAbsoluteEncoder =
        new AnalogInput(
            switch (module) {
              case 0 -> kFrontLeftAbsoluteEncoderPort;
              case 1 -> kFrontRightAbsoluteEncoderPort;
              case 2 -> kBackLeftAbsoluteEncoderPort;
              case 3 -> kBackRightAbsoluteEncoderPort;
              default -> 0;
            });
    driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    var driveConfig = new SparkMaxConfig();
    driveConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kDriveMotorCurrentLimit)
        .voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(kDriveEncoderPositionFactor)
        .velocityConversionFactor(kDriveEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            kDriveKp, 0.0,
            kDriveKd, 0.0);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / kOdometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        driveSpark,
        5,
        () ->
            driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(driveSpark, 5, () -> driveEncoder.setPosition(0.0));

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(kTurnInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kTurnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor(kTurnEncoderPositionFactor)
        .velocityConversionFactor(kTurnEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(kTurnPIDMinInput, kTurnPIDMaxInput)
        .pidf(kTurnKp, 0.0, kTurnKd, 0.0);
    // turnConfig
    //     .closedLoop
    //     .maxMotion
    //     .allowedClosedLoopError(turnMaxErrorTolerance)
    //     .maxVelocity(turnMaxVelocityRadPerSec)
    //     .maxAcceleration(turnMaxAccelerationRadPerSecSq);
    turnConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / kOdometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    // Reset neo relative encoder using absolute encoder position
    tryUntilOk(
        turnSpark,
        5,
        () ->
            turnEncoder.setPosition(
                turnAbsoluteEncoder.getVoltage()
                    / RobotController.getVoltage5V()
                    * kTurnAbsoluteEncoderPositionFactor));

    // Create odometry queues
    timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(driveSpark, driveEncoder::getPosition);
    turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);

    // Configure turn PID
    turnPID =
        new TunablePIDController(kTurnKp, 0.0, kTurnKd, kTurnMaxErrorTolerance, "Turn", false);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        driveSpark,
        new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    inputs.turnAbsolutePosition =
        new Rotation2d(
            turnAbsoluteEncoder.getVoltage()
                / RobotController.getVoltage5V()
                * kTurnAbsoluteEncoderPositionFactor);
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double ffVolts = kDriveKs * Math.signum(velocityRadPerSec) + kDriveKv * velocityRadPerSec;
    driveController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), kTurnPIDMinInput, kTurnPIDMaxInput);
    // turnController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
    turnSpark.setVoltage(turnPID.calculate(turnEncoder.getPosition(), setpoint));
  }
}
