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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO m_io;
  private final ModuleIOInputsAutoLogged m_inputs = new ModuleIOInputsAutoLogged();
  private final int m_index;

  private final LinearSystem<N1, N1, N1> m_drivePlant;
  private final LinearPlantInversionFeedforward<N1, N1, N1> m_driveFF;
  private final LinearQuadraticRegulator<N1, N1, N1> m_driveRegulator;
  private final LinearSystem<N2, N1, N2> m_turnPlant;
  private final LinearPlantInversionFeedforward<N2, N1, N2> m_turnFF;
  private final LinearQuadraticRegulator<N2, N1, N2> m_turnRegulator;
  private boolean isClosedLoop = false;
  private double m_driveFFVolts = 0.0;
  private PIDController m_driveController;
  private PIDController m_turnController;

  private final Alert m_driveDisconnectedAlert;
  private final Alert m_turnDisconnectedAlert;
  private SwerveModulePosition[] m_odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    m_io = io;
    m_index = index;

    SmartDashboard.putBoolean("DriveStateSpace", true);
    m_drivePlant = LinearSystemId.identifyVelocitySystem(kDriveKv, kDriveKa);
    m_driveFF = new LinearPlantInversionFeedforward<>(m_drivePlant, 0.02);
    m_driveRegulator =
        new LinearQuadraticRegulator<>(
            m_drivePlant, VecBuilder.fill(0.0001), VecBuilder.fill(12.0), 0.02);

    m_turnPlant = LinearSystemId.identifyPositionSystem(kTurnKv, kTurnKa);
    m_turnFF = new LinearPlantInversionFeedforward<>(m_turnPlant, 0.02);
    m_turnRegulator =
        new LinearQuadraticRegulator<>(
            m_turnPlant, VecBuilder.fill(0.2, 10.0), VecBuilder.fill(12.0), 0.02);

    m_driveController = new PIDController(kDriveSimP, 0.0, kDriveSimD);
    m_turnController = new PIDController(kTurnSimP, 0.0, kTurnSimD);
    m_turnController.enableContinuousInput(-Math.PI, Math.PI);

    m_driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(m_index) + ".",
            AlertType.kError);
    m_turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(m_index) + ".",
            AlertType.kError);
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(m_index), m_inputs);

    if (SmartDashboard.getBoolean("DriveStateSpace", true)) {
      if (isClosedLoop) {
        double driveSetpoint = m_driveController.getSetpoint();
        Vector<N1> nextDriveR = VecBuilder.fill(driveSetpoint);

        m_io.setDriveOpenLoop(
            m_driveRegulator
                .calculate(VecBuilder.fill(m_inputs.driveVelocityRadPerSec), nextDriveR)
                .plus(m_driveFF.calculate(nextDriveR).plus(kDriveKs * Math.signum(driveSetpoint)))
                .get(0, 0));

        double current = m_inputs.turnAbsolutePosition.getRadians();
        double error = MathUtil.angleModulus(m_turnController.getSetpoint() - current);
        Vector<N2> nextTurnR = VecBuilder.fill(current + error, 0.0);

        m_io.setTurnOpenLoop(
            m_turnRegulator
                .calculate(VecBuilder.fill(current, m_inputs.turnVelocityRadPerSec), nextTurnR)
                .plus(m_turnFF.calculate(nextTurnR))
                .get(0, 0));
      } else {
        m_driveController.reset();
        m_turnController.reset();
      }
    } else {
      if (isClosedLoop) {
        m_io.setDriveOpenLoop(
            m_driveFFVolts + m_driveController.calculate(m_inputs.driveVelocityRadPerSec));
        m_io.setTurnOpenLoop(
            m_turnController.calculate(m_inputs.turnAbsolutePosition.getRadians()));
      } else {
        m_driveController.reset();
        m_turnController.reset();
      }
    }

    // Calculate positions for odometry
    int sampleCount = m_inputs.odometryTimestamps.length; // All signals are sampled together
    m_odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = m_inputs.odometryDrivePositionsRad[i] * kWheelRadiusMeters;
      Rotation2d angle = m_inputs.odometryTurnPositions[i];
      m_odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    m_driveDisconnectedAlert.set(!m_inputs.driveConnected);
    m_turnDisconnectedAlert.set(!m_inputs.turnConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(m_inputs.turnPosition);

    isClosedLoop = true;
    double velocityRadPerSec = state.speedMetersPerSecond / kWheelRadiusMeters;
    m_driveFFVolts = kDriveSimKs * Math.signum(velocityRadPerSec) + kDriveSimKv * velocityRadPerSec;
    m_driveController.setSetpoint(velocityRadPerSec);
    m_turnController.setSetpoint(
        MathUtil.angleModulus(state.angle.plus(m_io.getZeroRotation()).getRadians()));
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    isClosedLoop = false;
    m_io.setDriveOpenLoop(output);
    m_turnController.setSetpoint(MathUtil.angleModulus(m_io.getZeroRotation().getRadians()));
    m_io.setTurnOpenLoop(m_turnController.calculate(m_inputs.turnAbsolutePosition.getRadians()));
  }

  public void runTurnCharacterization(double output) {
    isClosedLoop = false;
    m_io.setTurnOpenLoop(output);
  }

  /** Runs the turn motor with the specified output. */
  public void runTurnOpenLoop(double output) {
    isClosedLoop = false;
    m_io.setTurnOpenLoop(output);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    m_io.setDriveOpenLoop(0.0);
    m_io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return m_inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return m_inputs.drivePositionRad * kWheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return m_inputs.driveVelocityRadPerSec * kWheelRadiusMeters;
  }

  /**
   * Returns the current rotation velocity of the module in rad per sec. Only used for configuration
   * of swerve setpoint generator.
   */
  public double getModuleRotationVelocityRadPerSec() {
    return m_inputs.turnVelocityRadPerSec;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return m_odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return m_inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return m_inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return m_inputs.driveVelocityRadPerSec;
  }
}
