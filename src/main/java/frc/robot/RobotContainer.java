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
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.io.beambreak.BeamBreakIO;
import frc.robot.io.beambreak.BeamBreakIOReal;
import frc.robot.io.beambreak.BeamBreakIOSim;
import frc.robot.io.operatorPanel.OperatorPanel;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Feeder.FeederConstants;
import frc.robot.subsystems.Feeder.FeederConstants.FeederState;
import frc.robot.subsystems.Feeder.FeederIO;
import frc.robot.subsystems.Feeder.FeederIOSim;
import frc.robot.subsystems.Feeder.FeederIOSpark;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;
import frc.robot.subsystems.SuperStructure.Arm.ArmIO;
import frc.robot.subsystems.SuperStructure.Arm.ArmIOSim;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIO;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIOSim;
import frc.robot.subsystems.SuperStructure.Elevator.ElevatorIOSpark;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.heimdall.HeimdallPoseController;
import frc.robot.subsystems.vision.heimdall.HeimdallPoseController.HeimdallOdometrySource;
import frc.robot.subsystems.vision.photon.Vision;
import frc.robot.subsystems.vision.photon.VisionConstants;
import frc.robot.subsystems.vision.photon.VisionIO;
import frc.robot.subsystems.vision.photon.VisionIO.CameraType;
import frc.robot.subsystems.vision.photon.VisionIOPhotonVision;
import frc.robot.subsystems.vision.photon.VisionIOPhotonVisionSim;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefLevel;
import frc.robot.util.GamePieces.GamePieceVisualizer;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
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
  //   private final Collector m_collector;
  //   private final EndEffector m_endEffector;
  private final Feeder m_feeder;

  // SIM
  private SwerveDriveSimulation m_driveSimulation = null;

  // Controller
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final OperatorPanel m_operatorPanel = new OperatorPanel(1);
  /** leftTrigger and button 1 false */
  private final Trigger m_algaeProcessorTrigger;
  /** leftTrigger and button 1 true */
  private final Trigger m_algaeNetTrigger;
  /** leftBumper and button 2 true */
  private final Trigger m_algaeReefTrigger;
  /** left bumper and button 2 false */
  private final Trigger m_algaeFloorTrigger;
  /** right trigger and button 2 true */
  private final Trigger m_manualTroughSuperStructureTrigger;
  /** a and button 4 true */
  private final Trigger m_manualTroughScoreTrigger;
  /** right trigger and button 4 false */
  private final Trigger m_automaticCoralScoreTrigger;
  /** bogus call button 8 */
  private final Trigger m_bogusCallTrigger;
  /** disable brake mode button 5 */
  private final Trigger m_disableBrakeModeTrigger;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> m_autoChooser;
  private final LoggedDashboardChooser<SuperStructureState> m_stateChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_algaeNetTrigger =
        m_driverController.leftTrigger(0.1).and(m_operatorPanel.getOverrideSwitch(0));
    m_algaeProcessorTrigger =
        m_driverController.leftTrigger(0.1).and(m_operatorPanel.getNegatedOverrideSwitch(0));
    m_algaeReefTrigger =
        m_driverController.leftBumper().debounce(0.1).and(m_operatorPanel.getOverrideSwitch(1));
    m_algaeFloorTrigger =
        m_driverController
            .leftBumper()
            .debounce(0.1)
            .and(m_operatorPanel.getNegatedOverrideSwitch(1));
    m_manualTroughSuperStructureTrigger =
        m_driverController.rightTrigger(0.1).and(m_operatorPanel.getOverrideSwitch(3));
    m_automaticCoralScoreTrigger =
        m_driverController.rightTrigger(0.1).and(m_operatorPanel.getNegatedOverrideSwitch(3));
    m_manualTroughScoreTrigger = m_driverController.a().and(m_operatorPanel.getOverrideSwitch(3));
    m_bogusCallTrigger = new Trigger(m_operatorPanel.getOverrideSwitch(7));
    m_disableBrakeModeTrigger = new Trigger(m_operatorPanel.getOverrideSwitch(4));

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
                m_poseController,
                (pose) -> {});
        m_vision =
            new Vision(
                m_drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.kCamera0Name,
                    VisionConstants.kRobotToCamera0,
                    CameraType.APRILTAG),
                new VisionIOPhotonVision(
                    VisionConstants.kCamera1Name,
                    VisionConstants.kRobotToCamera1,
                    CameraType.APRILTAG));
        m_superStructure = new SuperStructure(new ElevatorIOSpark(), new ArmIO() {});
        // m_collector =
        //     new Collector(
        //         new IntakeIOSpark(),
        //         new FeederIOSpark(),
        //         new BeamBreakIOReal(IntakeConstants.kBeamBreakId));
        m_feeder =
            new Feeder(new FeederIOSpark(), new BeamBreakIOReal(FeederConstants.kBeamBreakId));
        // m_endEffector =
        //     new EndEffector(
        //         new AlgaeClawIOSpark(),
        //         new CoralEjectorIOSpark(),
        //         new BeamBreakIOReal(AlgaeClawConstants.kBeamBreakId),
        //         new BeamBreakIOReal(CoralEjectorConstants.kBeamBreakId));

        break;

      case SIM:
        // create a maple-sim swerve drive simulation instance
        m_driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.kMapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(m_driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        m_drive =
            new Drive(
                new GyroIOSim(m_driveSimulation.getGyroSimulation()),
                new ModuleIOSim(m_driveSimulation.getModules()[0]),
                new ModuleIOSim(m_driveSimulation.getModules()[1]),
                new ModuleIOSim(m_driveSimulation.getModules()[2]),
                new ModuleIOSim(m_driveSimulation.getModules()[3]),
                m_poseController,
                m_driveSimulation::setSimulationWorldPose);
        m_vision =
            new Vision(
                m_drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.kCamera0Name,
                    VisionConstants.kRobotToCamera0,
                    m_driveSimulation::getSimulatedDriveTrainPose,
                    CameraType.APRILTAG),
                new VisionIOPhotonVisionSim(
                    VisionConstants.kCamera1Name,
                    VisionConstants.kRobotToCamera1,
                    m_driveSimulation::getSimulatedDriveTrainPose,
                    CameraType.APRILTAG));

        // the sim lags really badly if you use auto switch
        m_poseController.setMode(HeimdallOdometrySource.AUTO_SWITCH);
        m_superStructure = new SuperStructure(new ElevatorIOSim(), new ArmIOSim());
        // m_collector =
        //     new Collector(
        //         new IntakeIOSim(m_driveSimulation), new FeederIOSim(), new BeamBreakIOSim());
        m_feeder = new Feeder(new FeederIOSim(), new BeamBreakIOSim());
        // m_endEffector =
        //     new EndEffector(
        //         new AlgaeClawIOSim(),
        //         new CoralEjectorIOSim(),
        //         new BeamBreakIOSim(),
        //         new BeamBreakIOSim());
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
                m_poseController,
                (pose) -> {});
        m_vision = new Vision(m_drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        m_superStructure = new SuperStructure(new ElevatorIO() {}, new ArmIO() {});
        // m_collector = new Collector(new IntakeIO() {}, new FeederIO() {}, new BeamBreakIO() {});
        m_feeder = new Feeder(new FeederIO() {}, new BeamBreakIO() {});
        // m_endEffector =
        //     new EndEffector(
        //         new AlgaeClawIO() {},
        //         new CoralEjectorIO() {},
        //         new BeamBreakIO() {},
        //         new BeamBreakIO() {});
        break;
    }

    // m_drive.setAdjustmentFactor(m_superStructure.getAdjustmentCoefficient());

    // Set up auto routines
    m_autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // m_autoChooser.addDefaultOption(
    //     "4_coral_auto",
    //     new RightAuto(m_drive, m_superStructure, m_endEffector, m_collector, m_vision));

    // Set up SysId routines
    m_autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));
    // m_autoChooser.addOption(
    //     "Module Turn Speed Characterization",
    //     DriveCommands.maxModuleRotationVelocityCharacterization(m_drive));
    // m_autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
    // m_autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // TODO not needed for now
    m_autoChooser.addOption(
        "Elevator SysId (Quasistatic Forward)",
        m_superStructure.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Elevator SysId (Quasistatic Reverse)",
        m_superStructure.elevatorSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    m_autoChooser.addOption(
        "Elevator SysId (Dynamic Forward)",
        m_superStructure.elevatorSysIdDynamic(SysIdRoutine.Direction.kForward));
    m_autoChooser.addOption(
        "Elevator SysId (Dynamic Reverse)",
        m_superStructure.elevatorSysIdDynamic(SysIdRoutine.Direction.kReverse));
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

    m_stateChooser = new LoggedDashboardChooser<>("StateChooser", new SendableChooser<>());

    for (SuperStructureState state : SuperStructureState.values()) {
      m_stateChooser.addDefaultOption(state.toString(), state);
    }

    // Configure the button bindings
    configureButtonBindings();

    // Set suppliers for the game piece visualizer
    GamePieceVisualizer.setRobotPoseSupplier(m_drive::getPose);
    // GamePieceVisualizer.setEndEffectorPoseSupplier(m_superStructure::getEndEffectorPose3d);
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

    // Disable PIDs on switch flip
    m_disableBrakeModeTrigger
        .onTrue(
            new InstantCommand(() -> m_superStructure.setBrakeMode(false)).ignoringDisable(true))
        .onFalse(
            new InstantCommand(() -> m_superStructure.setBrakeMode(true)).ignoringDisable(true));

    m_driverController
        .a()
        .whileTrue(
            new DeferredCommand(
                () ->
                    DriveCommands.pathfindThenPreciseAlign(
                        m_drive,
                        FieldConstants.Reef.branchPositions
                                .get(m_operatorPanel.getReefTarget())
                                .get(ReefLevel.fromLevel(m_operatorPanel.getReefLevel()))
                            ::toPose2d),
                Set.of(m_drive)));

    m_driverController
        .x()
        .onTrue(new InstantCommand(() -> m_superStructure.setElevatorGoal(ElevatorLevel.ALGAE_L2)));
    m_driverController
        .y()
        .onTrue(new InstantCommand(() -> m_superStructure.setElevatorGoal(ElevatorLevel.ALGAE_L3)));

    // m_driverController
    //     .x()
    //     .whileTrue(
    //         new StartEndCommand(
    //             () -> m_superStructure.runElevatorOpenLoop(-12.0),
    //             () -> m_superStructure.runElevatorOpenLoop(0.0),
    //             m_superStructure));
    // m_driverController
    //     .y()
    //     .whileTrue(
    //         new StartEndCommand(
    //             () -> m_superStructure.runElevatorOpenLoop(12.0),
    //             () -> m_superStructure.runElevatorOpenLoop(0.0),
    //             m_superStructure));

    m_driverController
        .b()
        .whileTrue(
            new StartEndCommand(
                () -> m_feeder.setFeederState(FeederState.FEEDING),
                () -> m_feeder.setFeederState(FeederState.IDLE),
                m_feeder));

    // ============================================================================
    // vvvvvvvvvvvvvvvvvvvvvvvvv TELEOP CONTROLLER BINDS vvvvvvvvvvvvvvvvvvvvvvvvv
    // ============================================================================

    // INTAKE CORAL
    // m_driverController
    //     // this is still not perfect but good enough for now
    //     // the arm transitioning to INTAKE should finish before a coral could possibly be intaked
    //     // fully
    //     .rightBumper()
    //     .onTrue(
    //         new SuperStructureCommand(m_superStructure, () -> SuperStructureState.INTAKE_CORAL)
    //             .unless(m_endEffector::isCoralHeld))
    //     .whileTrue(
    //         new ParallelDeadlineGroup(
    //                 new IntakeCoralCommand(m_collector, m_endEffector),
    //                 DriveCommands.driveToGamePiece(
    //                     m_drive,
    //                     () -> -m_driverController.getLeftY(),
    //                     () -> -m_driverController.getLeftX(),
    //                     () -> -m_driverController.getRightX(),
    //                     () -> m_vision.getTargetX(2).getRadians(),
    //                     true))
    //             .unless(m_endEffector::isCoralHeld))
    //     .onFalse(new SuperStructureCommand(m_superStructure, () -> SuperStructureState.IDLE));

    // EJECT GAME PIECE
    // m_driverController
    //     .b()
    //     .whileTrue(
    //         new SuperStructureCommand(m_superStructure, () -> SuperStructureState.INTAKE_CORAL)
    //             .andThen(new EjectIntakeCommand(m_collector)))
    //     .onFalse(new SuperStructureCommand(m_superStructure, () -> SuperStructureState.IDLE));
    // m_driverController.x().whileTrue(new ScoreAlgaeCommand(m_endEffector));

    // // SCORE CORAL
    // m_automaticCoralScoreTrigger
    //     .whileTrue(
    //         new AutoScoreCoralAtBranchCommand(
    //                 m_drive, m_superStructure, m_endEffector, m_operatorPanel::getTargetPose)
    //             .unless(() -> !m_endEffector.isCoralHeld()))
    //     .onFalse(new ResetSuperStructureCommand(m_drive, m_superStructure));

    // INTAKE ALGAE REEF
    // m_algaeReefTrigger
    //     .whileTrue(
    //         new AutoIntakeAlgaeReefCommand(
    //             m_drive, m_superStructure, m_endEffector, () -> m_drive.getPose()))
    //     .onFalse(
    //         new SequentialCommandGroup(
    //             new InstantCommand(
    //                 () ->
    // LEDSubsystem.getInstance().setStates(LEDStates.RESETTING_SUPERSTRUCTURE)),
    //             new WaitUntilFarFromCommand(m_drive::getPose, 0.5)
    //                 .deadlineFor(
    //                     new RunCommand(() -> m_drive.runVelocity(new ChassisSpeeds(-1, 0, 0))))
    //                 .unless(() -> DriverStation.isTest()),
    //             new SuperStructureCommand(m_superStructure, () ->
    // SuperStructureState.INTAKE_CORAL),
    //             new InstantCommand(() -> LEDSubsystem.getInstance().setStates(LEDStates.IDLE))));

    // INTAKE ALGAE FLOOR
    // m_algaeFloorTrigger
    //     .whileTrue(
    //         new SuperStructureCommand(
    //                 m_superStructure, () -> SuperStructureState.INTAKE_ALGAE_FLOOR)
    //             .alongWith(new IntakeAlgaeCommand(m_endEffector)))
    //     .onFalse(new SuperStructureCommand(m_superStructure, () -> SuperStructureState.IDLE));

    // // SCORE ALGAE PROCESSOR
    // m_algaeProcessorTrigger
    //     .whileTrue(
    //         new ScoreAlgaeProcessor(m_drive, m_superStructure, m_endEffector)
    //             .unless(() -> !m_endEffector.isAlgaeHeld()))
    //     .onFalse(new ResetSuperStructureCommand(m_drive, m_superStructure));

    // SCORE ALGAE NET
    // m_algaeNetTrigger
    //     .whileTrue(
    //         new ScoreAlgaeNet(m_drive, m_superStructure, m_endEffector)
    //             .unless(() -> !m_endEffector.isAlgaeHeld()))
    //     .onFalse(new ResetSuperStructureCommand(m_drive, m_superStructure));

    // MANUAL TROUGH SUPERSTRUCTURE
    // m_manualTroughSuperStructureTrigger
    //     .whileTrue(
    //         new SuperStructureCommand(m_superStructure, () ->
    // SuperStructureState.SCORE_CORAL_L1))
    //     .onFalse(new SuperStructureCommand(m_superStructure, () -> SuperStructureState.IDLE));

    // MANUAL CORAL SCORE
    // m_manualTroughScoreTrigger.onTrue(new ScoreCoralCommand(m_endEffector));

    // BOGUS CALL
    m_bogusCallTrigger
        .onTrue(
            new InstantCommand(() -> LEDSubsystem.getInstance().setStates(LEDStates.BOGUS_CALL)))
        .onFalse(new InstantCommand(() -> LEDSubsystem.getInstance().setStates(LEDStates.IDLE)));

    // ============================================================================
    // ^^^^^^^^^^^^^^^^^^^^^^^^^ TELEOP CONTROLLER BINDS ^^^^^^^^^^^^^^^^^^^^^^^^^
    // ============================================================================
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.kCurrentMode != Constants.Mode.SIM) return;

    m_drive.setPose(new Pose2d(3, 3, new Rotation2d()));

    // Reset GamePieceVisualizer (and SimulatedArena)
    GamePieceVisualizer.resetFieldGamePieces();
  }

  public void updateSimulation() {
    if (Constants.kCurrentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", m_driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
