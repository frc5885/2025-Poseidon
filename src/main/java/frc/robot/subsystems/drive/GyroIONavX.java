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

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS m_navX = new AHRS(NavXComType.kUSB1, (byte) kOdometryFrequency);
  private final Queue<Double> m_yawPositionQueue;
  private final Queue<Double> m_yawTimestampQueue;

  public GyroIONavX() {
    m_yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    m_yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(m_navX::getAngle);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = m_navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(-m_navX.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-m_navX.getRawGyroZ());

    inputs.odometryYawTimestamps =
        m_yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        m_yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    m_yawTimestampQueue.clear();
    m_yawPositionQueue.clear();
  }

  @Override
  public void resetGyro() {
    m_navX.reset();
  }
}
