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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Collector.Collector;
import frc.robot.subsystems.Collector.Feeder.FeederIO;
import frc.robot.subsystems.Collector.Feeder.FeederIOSim;
import frc.robot.subsystems.Collector.Feeder.FeederIOSpark;
import frc.robot.subsystems.Collector.Intake.IntakeIO;
import frc.robot.subsystems.Collector.Intake.IntakeIOSim;
import frc.robot.subsystems.Collector.Intake.IntakeIOSpark;
import frc.robot.subsystems.SuperStructure.Arm.ArmIO;
import frc.robot.subsystems.SuperStructure.Arm.ArmIOSim;
import frc.robot.subsystems.SuperStructure.Arm.ArmIOSpark;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIO;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIOSim;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIOSpark;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.heimdall.HeimdallPoseController;
import frc.robot.subsystems.vision.heimdall.HeimdallPoseController.HeimdallOdometrySource;
import frc.robot.subsystems.vision.photon.Vision;
import frc.robot.subsystems.vision.photon.VisionConstants;
import frc.robot.subsystems.vision.photon.VisionIO;
import frc.robot.subsystems.vision.photon.VisionIOPhotonVision;
import frc.robot.subsystems.vision.photon.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final HeimdallPoseController poseController;
  private final SuperStructure m_superStructure;
  private final Collector m_collector;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    poseController = new HeimdallPoseController(HeimdallOdometrySource.AUTO_SWITCH);
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                poseController);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1));
        m_superStructure = new SuperStructure(new ElevatorIOSpark(), new ArmIOSpark());
        m_collector = new Collector(new IntakeIOSpark(), new FeederIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                poseController);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        // the sim lags really badly if you use auto switch
        poseController.setMode(HeimdallOdometrySource.ONLY_APRILTAG_ODOMETRY);
        m_superStructure = new SuperStructure(new ElevatorIOSim(), new ArmIOSim());
        m_collector = new Collector(new IntakeIOSim(), new FeederIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                poseController);
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        m_superStructure = new SuperStructure(new ElevatorIO() {}, new ArmIO() {});
        m_collector = new Collector(new IntakeIO() {}, new FeederIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Module Turn Speed Characterization",
        DriveCommands.maxModuleRotationVelocityCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // TODO not needed for now
    // autoChooser.addOption(
    //     "Elevator SysId (Quasistatic Forward)",
    //     m_superStructure.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Elevator SysId (Quasistatic Reverse)",
    //     m_superStructure.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Elevator SysId (Dynamic Forward)",
    //     m_superStructure.elevatorSysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Elevator SysId (Dynamic Reverse)",
    //     m_superStructure.elevatorSysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Arm SysId (Quasistatic Forward)",
    //     m_superStructure.armSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Arm SysId (Quasistatic Reverse)",
    //     m_superStructure.armSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Arm SysId (Dynamic Forward)",
    //     m_superStructure.armSysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Arm SysId (Dynamic Reverse)",
    //     m_superStructure.armSysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.resetGyro();
                      Pose2d newPose = new Pose2d(0, 0, new Rotation2d());
                      drive.setPose(newPose);
                    },
                    drive)
                .ignoringDisable(true));

    controller.y().onTrue(new InstantCommand(() -> poseController.forceSyncQuest()));

    // elevator testing
    new JoystickButton(new GenericHID(1), 1)
        .onTrue(
            Commands.runOnce(
                () ->
                    m_superStructure.setElevatorLevel(
                        switch (m_superStructure.getElevatorLevel()) {
                          case L1 -> ElevatorLevel.L2;
                          case L2 -> ElevatorLevel.L3;
                          case L3 -> ElevatorLevel.L4;
                          case L4 -> ElevatorLevel.L1;
                          default -> ElevatorLevel.L1;
                        }),
                m_superStructure));

    // arm testing
    new JoystickButton(new GenericHID(1), 4)
        .onTrue(
            Commands.runOnce(
                () ->
                    m_superStructure.setArmGoal(
                        switch (m_superStructure.getArmGoal()) {
                          case STOW -> ArmGoals.RAISED;
                          case RAISED -> ArmGoals.SETPOINT;
                          case SETPOINT -> ArmGoals.STOW;
                          default -> ArmGoals.STOW;
                        }),
                m_superStructure));

    new JoystickButton(new GenericHID(1), 2)
        .whileTrue(
            new StartEndCommand(
                () -> m_collector.runIntake(12), () -> m_collector.stopIntake(), m_collector));

    new JoystickButton(new GenericHID(1), 3)
        .whileTrue(
            new StartEndCommand(
                () -> m_collector.runFeeder(12), () -> m_collector.stopFeeder(), m_collector));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
