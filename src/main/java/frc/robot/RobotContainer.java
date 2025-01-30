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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.QuestNav.QuestNav;
import frc.robot.subsystems.drive.QuestNav.QuestNavIO;
import frc.robot.subsystems.drive.QuestNav.QuestNavIOReal;
import frc.robot.subsystems.drive.QuestNav.QuestNavIOSim;
import frc.robot.subsystems.simplemanipulator.ManipulatorConstants.ElevatorConstants;
import frc.robot.subsystems.simplemanipulator.ManipulatorConstants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.simplemanipulator.SimpleManipulator;
import frc.robot.subsystems.simplemanipulator.elevator.ElevatorIO;
import frc.robot.subsystems.simplemanipulator.elevator.ElevatorIOSim;
import frc.robot.subsystems.simplemanipulator.elevator.ElevatorIOSpark;
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
  private final QuestNav questNav;
  private final SimpleManipulator m_manipulator;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        questNav = new QuestNav(new QuestNavIOReal(), drive.getPose());
        m_manipulator =
            new SimpleManipulator(new ElevatorIOSpark(ElevatorConstants.kElevatorSparkId));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        questNav = new QuestNav(new QuestNavIOSim(drive::getPose) {}, drive.getPose());
        m_manipulator = new SimpleManipulator(new ElevatorIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        questNav = new QuestNav(new QuestNavIO() {}, drive.getPose());
        m_manipulator = new SimpleManipulator(new ElevatorIO() {});
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
    //     m_manipulator.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Elevator SysId (Quasistatic Reverse)",
    //     m_manipulator.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Elevator SysId (Dynamic Forward)",
    //     m_manipulator.elevatorSysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Elevator SysId (Dynamic Reverse)",
    //     m_manipulator.elevatorSysIdDynamic(SysIdRoutine.Direction.kReverse));

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
                      questNav.setRobotPose(newPose);
                    },
                    drive)
                .ignoringDisable(true));

    controller.y().onTrue(new InstantCommand(() -> questNav.setRobotPose(drive.getPose())));

    // elevator testing
    new JoystickButton(new GenericHID(1), 1)
        .onTrue(
            Commands.runOnce(
                () ->
                    m_manipulator.setElevatorLevel(
                        switch (m_manipulator.getElevatorLevel()) {
                          case L1 -> ElevatorLevel.L2;
                          case L2 -> ElevatorLevel.L3;
                          case L3 -> ElevatorLevel.L4;
                          case L4 -> ElevatorLevel.L1;
                          default -> ElevatorLevel.L1;
                        }),
                m_manipulator));
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
