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

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.kMaxAccelerationMetersPerSecSq;
import static frc.robot.subsystems.drive.DriveConstants.kMaxAngularAccelerationRadiansPerSecSq;
import static frc.robot.subsystems.drive.DriveConstants.kMaxAngularSpeedRadiansPerSec;
import static frc.robot.subsystems.drive.DriveConstants.kMaxSpeedMetersPerSec;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ChassisTrapezoidalController;
import frc.robot.util.TunablePIDController;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
  private static final double kDeadband = 0.1;
  private static final double kAngleKp = 2.5;
  private static final double kAngleKd = 0.0;
  private static final double kTranslateKp = 2.5;
  private static final double kTranslateKd = 0.0;
  private static final double kFfStartDelay = 2.0; // Secs
  private static final double kFfRampRate = 0.1; // Volts/Sec
  private static final double kWheelRadiusMaxVelocity = 0.25; // Rad/Sec
  private static final double kWheelRadiusRampRate = 0.05; // Rad/Sec^2
  private static final double kAngleTolerance = 0.02;
  private static final double kTranslationTolerance = 0.02;

  // Create PID controllers
  private static TunablePIDController angleController;
  private static TunablePIDController xController;
  private static TunablePIDController yController;
  private static ChassisTrapezoidalController chassisController;
  // Static initialization block
  static {
    angleController =
        new TunablePIDController(
            kAngleKp, 0.0, kAngleKd, kAngleTolerance, "DriveAngleController", true);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    xController =
        new TunablePIDController(
            kTranslateKp, 0.0, kTranslateKd, kTranslationTolerance, "DriveXController", true);
    yController =
        new TunablePIDController(
            kTranslateKp, 0.0, kTranslateKd, kTranslationTolerance, "DriveYController", true);
    chassisController =
        new ChassisTrapezoidalController(
            new TrapezoidProfile.Constraints(kMaxSpeedMetersPerSec, kMaxAccelerationMetersPerSecSq),
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSec, kMaxAngularAccelerationRadiansPerSecSq),
            xController,
            yController,
            angleController);
  }

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), kDeadband);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), kDeadband);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Construct command
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Calculate angular speed
          double omega =
              angleController.calculate(
                  drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega);
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /** Robot relative drive command using PID for angular control. */
  public static Command driveToGamePiece(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier joystickRotSupplier,
      DoubleSupplier visionRotSupplier,
      boolean humanOperated) {

    // Construct command
    return Commands.run(
            () -> {
              boolean seesGamePiece = visionRotSupplier.getAsDouble() != 0.0;
              LEDSubsystem.getInstance().setSeesGamePiece(seesGamePiece);

              // Get linear velocity based on control mode
              Translation2d linearVelocity;
              if (humanOperated) {
                // Use joystick input for manual control
                linearVelocity =
                    getLinearVelocityFromJoysticks(
                        xSupplier.getAsDouble(), ySupplier.getAsDouble());
              } else {
                // Use vision feedback for autonomous control
                // Scale vision input down by dividing by PI to reduce sensitivity
                double yVelocity =
                    yController.calculate(visionRotSupplier.getAsDouble() / Math.PI, 0);
                linearVelocity = new Translation2d(xSupplier.getAsDouble(), -yVelocity);
              }

              // Process rotation input from joystick
              double joystickOmega =
                  MathUtil.applyDeadband(joystickRotSupplier.getAsDouble(), kDeadband);
              joystickOmega =
                  Math.copySign(
                      joystickOmega * joystickOmega, joystickOmega); // Square for fine control
              joystickOmega *= drive.getMaxAngularSpeedRadPerSec();

              // Calculate final rotation speed
              double omega;
              if (DriverStation.isTest()) {
                // Use direct joystick control in test mode
                omega = joystickOmega;
              } else {
                // Use vision-based alignment with joystick offset
                omega =
                    angleController.calculate(visionRotSupplier.getAsDouble() - joystickOmega, 0);
              }

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);

              if (humanOperated) {
                // field oriented (teleop)
                boolean isFlipped =
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red;
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds,
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
              } else {
                // robot oriented (auto)
                drive.runVelocity(speeds);
              }
            },
            drive)
        .finallyDo(() -> LEDSubsystem.getInstance().setSeesGamePiece(false));
  }

  /** Robot relative drive command for precise reef faces aligning. */
  public static Command preciseChassisAlign(Drive drive, Supplier<Pose2d> targetPose) {

    // Construct command
    return Commands.run(
            () -> {
              // Convert to field relative speeds & send command
              ChassisSpeeds speeds = chassisController.calculate(drive.getPose());
              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
            },
            drive)
        .until(chassisController::isGoalAchieved)
        .beforeStarting(
            () -> {
              chassisController.setGoalPose(AllianceFlipUtil.apply(targetPose.get()));
              chassisController.setCurrentState(drive.getPose(), drive.getChassisSpeeds());
            })
        .finallyDo(drive::stop);
  }

  /**
   * Pathfinding to pose, followed by a smooth transition to precise chassis align
   *
   * @param drive Drive subsystem
   * @param targetPose Target pose
   */
  public static Command pathfindThenPreciseAlign(Drive drive, Supplier<Pose2d> targetPose) {
    return new DeferredCommand(() -> drive.getPathFollowCommand(targetPose), Set.of(drive));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(kFfStartDelay),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * kFfRampRate;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(kWheelRadiusRampRate);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(kWheelRadiusMaxVelocity);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.kDriveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  /** Measures the max rotation velocity of a swerve module in radians per second */
  public static Command maxModuleRotationVelocityCharacterization(Drive drive) {
    MaxTurnVelocityState state = new MaxTurnVelocityState();

    return Commands.parallel(
        // Turn motors control sequence: run turn motors at full voltage.
        Commands.run(
            () -> {
              drive.runTurnOpenLoop(12.0);
            },
            drive),

        // Measurement sequence: record maximum turn velocity for each module.
        Commands.sequence(
            // Wait a moment for the modules to spin up.
            Commands.waitSeconds(1.0),

            // Continuously record the maximum measured velocity.
            Commands.run(
                    () -> {
                      // For each of the four modules...
                      for (int i = 0; i < 4; i++) {
                        double currentVelocity =
                            Math.abs(drive.getModuleRotationVelocityRadPerSec(i));
                        // Update if the current velocity is higher than the stored maximum.
                        state.maxVelocities[i] = Math.max(state.maxVelocities[i], currentVelocity);
                      }
                    })
                // When the command ends (e.g. when cancelled), print the results.
                .finallyDo(
                    interrupted -> {
                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Swerve Module Max Turn Velocity Results **********");
                      for (int i = 0; i < 4; i++) {
                        System.out.println(
                            "\tModule "
                                + i
                                + ": "
                                + formatter.format(state.maxVelocities[i])
                                + " rad/s");
                      }
                    })));
  }

  private static class MaxTurnVelocityState {
    // Array for 4 modules. Initialized to 0.
    public double[] maxVelocities = new double[4];
  }
}
