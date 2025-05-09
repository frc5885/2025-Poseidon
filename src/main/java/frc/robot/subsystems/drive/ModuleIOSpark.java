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
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOSpark implements ModuleIO {
  private final Rotation2d m_zeroRotation;

  // Hardware objects
  private final SparkMax m_driveSpark;
  private final SparkMax m_turnSpark;
  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turnEncoder;
  private final SparkAnalogSensor m_turnAbsoluteEncoder;

  // Queue inputs from odometry thread
  private final Queue<Double> m_timestampQueue;
  private final Queue<Double> m_drivePositionQueue;
  private final Queue<Double> m_turnPositionQueue;

  // Connection debouncers
  private final Debouncer m_driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer m_turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIOSpark(int module) {
    m_zeroRotation =
        switch (module) {
          case 0 -> kFrontLeftZeroRotation;
          case 1 -> kFrontRightZeroRotation;
          case 2 -> kBackLeftZeroRotation;
          case 3 -> kBackRightZeroRotation;
          default -> new Rotation2d();
        };
    m_driveSpark =
        new SparkMax(
            switch (module) {
              case 0 -> kFrontLeftDriveCanId;
              case 1 -> kFrontRightDriveCanId;
              case 2 -> kBackLeftDriveCanId;
              case 3 -> kBackRightDriveCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    m_turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> kFrontLeftTurnCanId;
              case 1 -> kFrontRightTurnCanId;
              case 2 -> kBackLeftTurnCanId;
              case 3 -> kBackRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    m_driveEncoder = m_driveSpark.getEncoder();
    m_turnEncoder = m_turnSpark.getEncoder();
    m_turnAbsoluteEncoder = m_turnSpark.getAnalog();

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
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / kOdometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        m_driveSpark,
        5,
        () ->
            m_driveSpark.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(m_driveSpark, 5, () -> m_driveEncoder.setPosition(0.0));

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
        .analogSensor
        .inverted(kTurnEncoderInverted)
        .positionConversionFactor(kTurnAbsoluteEncoderPositionFactor)
        .velocityConversionFactor(kTurnAbsoluteEncoderVelocityFactor);
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
        m_turnSpark,
        5,
        () ->
            m_turnSpark.configure(
                turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    // Reset neo relative encoder using absolute encoder position
    double[] turnAbsPos = new double[1];
    ifOk(m_turnSpark, m_turnAbsoluteEncoder::getPosition, (value) -> turnAbsPos[0] = value);
    tryUntilOk(m_turnSpark, 5, () -> m_turnEncoder.setPosition(turnAbsPos[0]));

    // Create odometry queues
    m_timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    m_drivePositionQueue =
        SparkOdometryThread.getInstance().registerSignal(m_driveSpark, m_driveEncoder::getPosition);
    m_turnPositionQueue =
        SparkOdometryThread.getInstance().registerSignal(m_turnSpark, m_turnEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive inputs
    sparkStickyFault = false;
    ifOk(m_driveSpark, m_driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
    ifOk(
        m_driveSpark,
        m_driveEncoder::getVelocity,
        (value) -> inputs.driveVelocityRadPerSec = value);
    ifOk(
        m_driveSpark,
        new DoubleSupplier[] {m_driveSpark::getAppliedOutput, m_driveSpark::getBusVoltage},
        (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
    ifOk(m_driveSpark, m_driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
    inputs.driveConnected = m_driveConnectedDebounce.calculate(!sparkStickyFault);

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        m_turnSpark,
        m_turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(m_zeroRotation));
    ifOk(
        m_turnSpark,
        m_turnAbsoluteEncoder::getPosition,
        (value) -> inputs.turnAbsolutePosition = new Rotation2d(value));
    ifOk(m_turnSpark, m_turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        m_turnSpark,
        new DoubleSupplier[] {m_turnSpark::getAppliedOutput, m_turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(m_turnSpark, m_turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = m_turnConnectedDebounce.calculate(!sparkStickyFault);

    // Update odometry inputs
    inputs.odometryTimestamps =
        m_timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        m_drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryTurnPositions =
        m_turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value).minus(m_zeroRotation))
            .toArray(Rotation2d[]::new);
    m_timestampQueue.clear();
    m_drivePositionQueue.clear();
    m_turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    m_driveSpark.setVoltage(output);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    m_turnSpark.setVoltage(output);
  }

  @Override
  public Rotation2d getZeroRotation() {
    return m_zeroRotation;
  }
}
