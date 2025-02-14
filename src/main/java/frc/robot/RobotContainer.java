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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Collector.Collector;
import frc.robot.subsystems.Collector.Feeder.FeederIO;
import frc.robot.subsystems.Collector.Feeder.FeederIOSim;
import frc.robot.subsystems.Collector.Feeder.FeederIOSpark;
import frc.robot.subsystems.Collector.Intake.IntakeIO;
import frc.robot.subsystems.Collector.Intake.IntakeIOSim;
import frc.robot.subsystems.Collector.Intake.IntakeIOSpark;
import frc.robot.subsystems.EndEffector.AlgaeClaw.AlgaeClawIO;
import frc.robot.subsystems.EndEffector.AlgaeClaw.AlgaeClawIOSim;
import frc.robot.subsystems.EndEffector.AlgaeClaw.AlgaeClawIOSpark;
import frc.robot.subsystems.EndEffector.CoralEjector.CoralEjectorIO;
import frc.robot.subsystems.EndEffector.CoralEjector.CoralEjectorIOSim;
import frc.robot.subsystems.EndEffector.CoralEjector.CoralEjectorIOSpark;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.SuperStructure.Arm.ArmIO;
import frc.robot.subsystems.SuperStructure.Arm.ArmIOSim;
import frc.robot.subsystems.SuperStructure.Arm.ArmIOSpark;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIO;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIOSim;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIOSpark;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.SuperStructure.Wrist.WristIO;
import frc.robot.subsystems.SuperStructure.Wrist.WristIOSim;
import frc.robot.subsystems.SuperStructure.Wrist.WristIOSpark;
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
import frc.robot.subsystems.vision.photon.VisionIOPhotonVisionSim.CameraType;
import frc.robot.util.GamePieces.GamePieceVisualizer;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_drive;
  private final Vision m_vision;
  private final HeimdallPoseController m_poseController;
  private final SuperStructure m_superStructure;
  private final Collector m_collector;
  private final EndEffector m_endEffector;

  // Controller
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> m_autoChooser;
  private final LoggedDashboardChooser<SuperStructureState> m_stateChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_poseController = new HeimdallPoseController(HeimdallOdometrySource.AUTO_SWITCH);
    switch (Constants.kCurrentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                m_poseController);
        m_vision =
            new Vision(
                m_drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.kCamera0Name, VisionConstants.kRobotToCamera0),
                new VisionIOPhotonVision(
                    VisionConstants.kCamera1Name, VisionConstants.kRobotToCamera1));
        m_superStructure =
            new SuperStructure(new ElevatorIOSpark(), new ArmIOSpark(), new WristIOSpark());
        m_collector = new Collector(new IntakeIOSpark(), new FeederIOSpark());
        m_endEffector = new EndEffector(new AlgaeClawIOSpark(), new CoralEjectorIOSpark());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                m_poseController);
        m_vision =
            new Vision(
                m_drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.kCamera0Name,
                    VisionConstants.kRobotToCamera0,
                    m_drive::getPose,
                    CameraType.AprilTag),
                new VisionIOPhotonVisionSim(
                    VisionConstants.kCamera1Name,
                    VisionConstants.kRobotToCamera1,
                    m_drive::getPose,
                    CameraType.AprilTag),
                new VisionIOPhotonVisionSim(
                    VisionConstants.kCamera2Name,
                    VisionConstants.kRobotToCamera2,
                    m_drive::getPose,
                    CameraType.Coral));
        // the sim lags really badly if you use auto switch
        m_poseController.setMode(HeimdallOdometrySource.ONLY_APRILTAG_ODOMETRY);
        m_superStructure =
            new SuperStructure(new ElevatorIOSim(), new ArmIOSim(), new WristIOSim());
        m_collector = new Collector(new IntakeIOSim(), new FeederIOSim());
        m_endEffector = new EndEffector(new AlgaeClawIOSim(), new CoralEjectorIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                m_poseController);
        m_vision = new Vision(m_drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        m_superStructure =
            new SuperStructure(new ElevatorIO() {}, new ArmIO() {}, new WristIO() {});
        m_collector = new Collector(new IntakeIO() {}, new FeederIO() {});

        m_endEffector = new EndEffector(new AlgaeClawIO() {}, new CoralEjectorIO() {});
        break;
    }

    // Set up auto routines
    m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    m_autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));
    m_autoChooser.addOption(
        "Module Turn Speed Characterization",
        DriveCommands.maxModuleRotationVelocityCharacterization(m_drive));
    m_autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // TODO not needed for now
    // m_autoChooser.addOption(
    //     "Elevator SysId (Quasistatic Forward)",
    //     m_superStructure.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_autoChooser.addOption(
    //     "Elevator SysId (Quasistatic Reverse)",
    //     m_superStructure.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_autoChooser.addOption(
    //     "Elevator SysId (Dynamic Forward)",
    //     m_superStructure.elevatorSysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_autoChooser.addOption(
    //     "Elevator SysId (Dynamic Reverse)",
    //     m_superStructure.elevatorSysIdDynamic(SysIdRoutine.Direction.kReverse));
    // m_autoChooser.addOption(
    //     "Arm SysId (Quasistatic Forward)",
    //     m_superStructure.armSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_autoChooser.addOption(
    //     "Arm SysId (Quasistatic Reverse)",
    //     m_superStructure.armSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_autoChooser.addOption(
    //     "Arm SysId (Dynamic Forward)",
    //     m_superStructure.armSysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_autoChooser.addOption(
    //     "Arm SysId (Dynamic Reverse)",
    //     m_superStructure.armSysIdDynamic(SysIdRoutine.Direction.kReverse));
    // m_autoChooser.addOption(
    //     "Wrist SysId (Quasistatic Forward)",
    //     m_superStructure.wristSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_autoChooser.addOption(
    //     "Wrist SysId (Quasistatic Reverse)",
    //     m_superStructure.wristSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_autoChooser.addOption(
    //     "Wrist SysId (Dynamic Forward)",
    //     m_superStructure.wristSysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_autoChooser.addOption(
    //     "Wrist SysId (Dynamic Reverse)",
    //     m_superStructure.wristSysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_stateChooser = new LoggedDashboardChooser<>("StateChooser", new SendableChooser<>());

    for (SuperStructureState state : SuperStructureState.values()) {
      m_stateChooser.addDefaultOption(state.toString(), state);
    }

    // Configure the button bindings
    configureButtonBindings();

    // Set suppliers for the game piece visualizer
    GamePieceVisualizer.setRobotPoseSupplier(m_drive::getPose);
    GamePieceVisualizer.setEndEffectorPoseSupplier(m_superStructure::getEndEffectorPose3d);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // Lock to 0° when A button is held
    m_driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_drive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    m_driverController.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

    // Reset gyro to 0° when leftStick is pressed
    m_driverController
        .leftStick()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      m_drive.resetGyro();
                      Pose2d newPose = new Pose2d(0, 0, new Rotation2d());
                      m_drive.setPose(newPose);
                    },
                    m_drive)
                .ignoringDisable(true));

    m_driverController.y().onTrue(new InstantCommand(() -> m_poseController.forceSyncQuest()));

    // superstructure testing
    m_driverController
        .b()
        .onTrue(
            new InstantCommand(
                () -> m_superStructure.setSuperStructureGoal(m_stateChooser.get()).schedule(),
                m_superStructure));

    // new JoystickButton(new GenericHID(1), 2)
    //     .whileTrue(
    //         new StartEndCommand(
    //             () -> m_collector.runIntake(12), () -> m_collector.stopIntake(), m_collector));

    // new JoystickButton(new GenericHID(1), 3)
    //     .whileTrue(
    //         new StartEndCommand(
    //             () -> m_collector.runFeeder(12), () -> m_collector.stopFeeder(), m_collector));

    // new JoystickButton(new GenericHID(1), 4)
    //     .whileTrue(
    //         new StartEndCommand(
    //             () -> m_endEffector.runCoralEjector(12),
    //             () -> m_endEffector.stopCoralEjector(),
    //             m_collector));

    // new JoystickButton(new GenericHID(1), 5)
    //     .whileTrue(
    //         new StartEndCommand(
    //             () -> m_endEffector.runAlgaeClaw(12),
    //             () -> m_endEffector.stopAlgaeClaw(),
    //             m_collector));
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }
}
