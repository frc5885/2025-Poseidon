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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.io.beambreak.BeamBreakIO;
import frc.robot.io.beambreak.BeamBreakIOReal;
import frc.robot.io.beambreak.BeamBreakIOSim;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.EndEffector.EndEffectorIO;
import frc.robot.subsystems.EndEffector.EndEffectorIOSim;
import frc.robot.subsystems.EndEffector.EndEffectorIOSpark;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Feeder.FeederConstants;
import frc.robot.subsystems.Feeder.FeederIO;
import frc.robot.subsystems.Feeder.FeederIOSim;
import frc.robot.subsystems.Feeder.FeederIOSpark;
import frc.robot.subsystems.SuperStructure.Arm.ArmIO;
import frc.robot.subsystems.SuperStructure.Arm.ArmIOSim;
import frc.robot.subsystems.SuperStructure.Arm.ArmIOSpark;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIO;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIOSim;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIOSpark;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.heimdall.HeimdallPoseController;
import frc.robot.subsystems.vision.photon.Vision;
import frc.robot.subsystems.vision.photon.VisionConstants;
import frc.robot.subsystems.vision.photon.VisionIO;
import frc.robot.subsystems.vision.photon.VisionIO.CameraType;
import frc.robot.subsystems.vision.photon.VisionIOPhotonVision;
import frc.robot.subsystems.vision.photon.VisionIOPhotonVisionSim;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * Factory class for creating subsystems based on the current robot mode. This centralizes all
 * mode-specific subsystem creation logic to keep RobotContainer clean and maintainable.
 */
public class SubsystemFactory {

  /**
   * Creates a Drive subsystem configured for the current robot mode.
   *
   * @param poseController The HeimdallPoseController for pose estimation
   * @param poseConsumer Consumer for pose updates (for simulation or no-op)
   * @param driveSimulation Supplier that provides the SwerveDriveSimulation (will be called only in
   *     SIM mode)
   * @return Configured Drive subsystem
   */
  public static Drive createDrive(
      HeimdallPoseController poseController,
      Consumer<Pose2d> poseConsumer,
      Supplier<SwerveDriveSimulation> driveSimulation) {
    switch (Constants.kCurrentMode) {
      case REAL:
        return new Drive(
            new GyroIONavX(),
            new ModuleIOSpark(0),
            new ModuleIOSpark(1),
            new ModuleIOSpark(2),
            new ModuleIOSpark(3),
            poseController,
            (pose) -> {});

      case SIM:
        SwerveDriveSimulation sim = driveSimulation.get();
        return new Drive(
            new GyroIOSim(sim.getGyroSimulation()),
            new ModuleIOSim(sim.getModules()[0]),
            new ModuleIOSim(sim.getModules()[1]),
            new ModuleIOSim(sim.getModules()[2]),
            new ModuleIOSim(sim.getModules()[3]),
            poseController,
            sim::setSimulationWorldPose);

      default:
        // Replayed robot, disable IO implementations
        return new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            poseController,
            (pose) -> {});
    }
  }

  /**
   * Creates a Vision subsystem configured for the current robot mode.
   *
   * @param visionMeasurementConsumer Consumer for vision measurements
   * @param driveSimulation Supplier that provides the SwerveDriveSimulation (will be called only in
   *     SIM mode)
   * @return Configured Vision subsystem
   */
  public static Vision createVision(
      Vision.VisionConsumer visionMeasurementConsumer,
      Supplier<SwerveDriveSimulation> driveSimulation) {
    switch (Constants.kCurrentMode) {
      case REAL:
        return new Vision(
            visionMeasurementConsumer,
            new VisionIOPhotonVision(
                VisionConstants.kCamera0Name, VisionConstants.kRobotToCamera0, CameraType.APRILTAG),
            new VisionIOPhotonVision(
                VisionConstants.kCamera1Name,
                VisionConstants.kRobotToCamera1,
                CameraType.APRILTAG));

      case SIM:
        SwerveDriveSimulation sim = driveSimulation.get();
        return new Vision(
            visionMeasurementConsumer,
            new VisionIOPhotonVisionSim(
                VisionConstants.kCamera0Name,
                VisionConstants.kRobotToCamera0,
                sim::getSimulatedDriveTrainPose,
                CameraType.APRILTAG),
            new VisionIOPhotonVisionSim(
                VisionConstants.kCamera1Name,
                VisionConstants.kRobotToCamera1,
                sim::getSimulatedDriveTrainPose,
                CameraType.APRILTAG));

      default:
        // Replayed robot, disable IO implementations
        return new Vision(visionMeasurementConsumer, new VisionIO() {}, new VisionIO() {});
    }
  }

  /**
   * Creates a SuperStructure subsystem configured for the current robot mode.
   *
   * @return Configured SuperStructure subsystem
   */
  public static SuperStructure createSuperStructure() {
    switch (Constants.kCurrentMode) {
      case REAL:
        return new SuperStructure(new ElevatorIOSpark(), new ArmIOSpark() {});

      case SIM:
        return new SuperStructure(new ElevatorIOSim(), new ArmIOSim());

      default:
        // Replayed robot, disable IO implementations
        return new SuperStructure(new ElevatorIO() {}, new ArmIO() {});
    }
  }

  /**
   * Creates a Feeder subsystem configured for the current robot mode.
   *
   * @return Configured Feeder subsystem
   */
  public static Feeder createFeeder() {
    switch (Constants.kCurrentMode) {
      case REAL:
        return new Feeder(new FeederIOSpark(), new BeamBreakIOReal(FeederConstants.kBeamBreakId));

      case SIM:
        return new Feeder(new FeederIOSim(), new BeamBreakIOSim());

      default:
        // Replayed robot, disable IO implementations
        return new Feeder(new FeederIO() {}, new BeamBreakIO() {});
    }
  }

  /**
   * Creates an EndEffector subsystem configured for the current robot mode.
   *
   * @return Configured EndEffector subsystem
   */
  public static EndEffector createEndEffector() {
    switch (Constants.kCurrentMode) {
      case REAL:
        return new EndEffector(new EndEffectorIOSpark());

      case SIM:
        return new EndEffector(new EndEffectorIOSim());

      default:
        // Replayed robot, disable IO implementations
        return new EndEffector(new EndEffectorIO() {});
    }
  }

  /**
   * Creates and initializes a SwerveDriveSimulation for simulation mode. Also adds it to the
   * SimulatedArena.
   *
   * @return Configured SwerveDriveSimulation, or null if not in SIM mode
   */
  public static SwerveDriveSimulation createDriveSimulation() {
    if (Constants.kCurrentMode != Constants.Mode.SIM) {
      return null;
    }

    // Create a maple-sim swerve drive simulation instance
    SwerveDriveSimulation driveSimulation =
        new SwerveDriveSimulation(
            DriveConstants.kMapleSimConfig, new Pose2d(3, 3, new Rotation2d()));

    // Add the simulated drivetrain to the simulation field
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

    return driveSimulation;
  }
}
