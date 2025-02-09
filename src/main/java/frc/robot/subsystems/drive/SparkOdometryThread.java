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

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
public class SparkOdometryThread {
  private final List<SparkBase> m_sparks = new ArrayList<>();
  private final List<DoubleSupplier> m_sparkSignals = new ArrayList<>();
  private final List<DoubleSupplier> m_genericSignals = new ArrayList<>();
  private final List<Queue<Double>> m_sparkQueues = new ArrayList<>();
  private final List<Queue<Double>> m_genericQueues = new ArrayList<>();
  private final List<Queue<Double>> m_timestampQueues = new ArrayList<>();

  private static SparkOdometryThread m_instance = null;
  private Notifier m_notifier = new Notifier(this::run);

  public static SparkOdometryThread getInstance() {
    if (m_instance == null) {
      m_instance = new SparkOdometryThread();
    }
    return m_instance;
  }

  private SparkOdometryThread() {
    m_notifier.setName("OdometryThread");
  }

  public void start() {
    if (m_timestampQueues.size() > 0) {
      m_notifier.startPeriodic(1.0 / DriveConstants.kOdometryFrequency);
    }
  }

  /** Registers a Spark signal to be read from the thread. */
  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.m_odometryLock.lock();
    try {
      m_sparks.add(spark);
      m_sparkSignals.add(signal);
      m_sparkQueues.add(queue);
    } finally {
      Drive.m_odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.m_odometryLock.lock();
    try {
      m_genericSignals.add(signal);
      m_genericQueues.add(queue);
    } finally {
      Drive.m_odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.m_odometryLock.lock();
    try {
      m_timestampQueues.add(queue);
    } finally {
      Drive.m_odometryLock.unlock();
    }
    return queue;
  }

  private void run() {
    // Save new data to queues
    Drive.m_odometryLock.lock();
    try {
      // Get sample timestamp
      double timestamp = RobotController.getFPGATime() / 1e6;

      // Read Spark values, mark invalid in case of error
      double[] sparkValues = new double[m_sparkSignals.size()];
      boolean isValid = true;
      for (int i = 0; i < m_sparkSignals.size(); i++) {
        sparkValues[i] = m_sparkSignals.get(i).getAsDouble();
        if (m_sparks.get(i).getLastError() != REVLibError.kOk) {
          isValid = false;
        }
      }

      // If valid, add values to queues
      if (isValid) {
        for (int i = 0; i < m_sparkSignals.size(); i++) {
          m_sparkQueues.get(i).offer(sparkValues[i]);
        }
        for (int i = 0; i < m_genericSignals.size(); i++) {
          m_genericQueues.get(i).offer(m_genericSignals.get(i).getAsDouble());
        }
        for (int i = 0; i < m_timestampQueues.size(); i++) {
          m_timestampQueues.get(i).offer(timestamp);
        }
      }
    } finally {
      Drive.m_odometryLock.unlock();
    }
  }
}
