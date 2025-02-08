// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffecter.CoralEjecter;

import org.littletonrobotics.junction.AutoLog;

public interface CoralEjecterIO {
  @AutoLog
  public class CoralEjecterInputs {
    public double positionRotations = 0.0;
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean CoralEjecterConnected = false;
  }

  double velocityRPM = 0;

  /** Updates the set of loggable inputs. */
  public default void updateInputs(CoralEjecterInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}
}
