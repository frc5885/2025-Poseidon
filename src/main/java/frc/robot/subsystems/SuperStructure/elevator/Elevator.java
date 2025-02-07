package frc.robot.subsystems.SuperStructure.elevator;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Elevator {
  private final ElevatorIO m_io;
  private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motor1DisconnectedAlert;
  private final Alert motor2DisconnectedAlert;

  private TrapezoidProfile m_elevatorProfile =
      new TrapezoidProfile(new Constraints(kElevatorMaxVelocity, kElevatorMaxAcceleration));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TunablePIDController m_elevatorController;
  private ElevatorFeedforward m_elevatorFeedforward;

  private final LoggedMechanism2d m_elevatorMech;
  private final LoggedMechanismRoot2d m_elevatorTrack;
  private final LoggedMechanismRoot2d m_elevatorRoot;

  private ElevatorLevel m_elevatorLevel = ElevatorLevel.L1;

  public Elevator(ElevatorIO io) {
    m_io = io;

    switch (Constants.currentMode) {
      case REAL:
        m_elevatorController =
            new TunablePIDController(
                elevatorKp, 0.0, elevatorKd, kElevatorErrorToleranceMeters, "ElevatorPID", true);
        m_elevatorFeedforward = new ElevatorFeedforward(elevatorKs, elevatorKg, elevatorKv);
        break;
      case SIM:
        m_elevatorController =
            new TunablePIDController(
                elevatorSimKp,
                0.0,
                elevatorSimKd,
                kElevatorErrorToleranceMeters,
                "ElevatorSimPID",
                true);
        m_elevatorFeedforward = new ElevatorFeedforward(0.0, elevatorSimKg, elevatorSimKv);
        break;
      case REPLAY:
        m_elevatorController =
            new TunablePIDController(
                elevatorSimKp,
                0.0,
                elevatorSimKd,
                kElevatorErrorToleranceMeters,
                "ElevatorReplayPID",
                true);
        m_elevatorFeedforward = new ElevatorFeedforward(0.0, elevatorSimKg, elevatorSimKv);
        break;
      default:
        m_elevatorController = new TunablePIDController(0.0, 0.0, 0.0, 0.0, "", false);
        m_elevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);
        break;
    }

    motor1DisconnectedAlert = new Alert("Elevator motor1 disconnected", AlertType.kError);
    motor2DisconnectedAlert = new Alert("Elevator motor2 disconnected", AlertType.kError);

    m_elevatorMech = new LoggedMechanism2d(0.2, 2.0);
    m_elevatorTrack = m_elevatorMech.getRoot("ElevatorTrack", 0.07, 0.15);
    m_elevatorTrack.append(
        new LoggedMechanismLigament2d("ElevatorTrack", kElevatorUpperBoundMeters, 90.0));
    m_elevatorRoot = m_elevatorMech.getRoot("ElevatorRoot", 0.13, 0.15);
    m_elevatorRoot.append(
        new LoggedMechanismLigament2d("Elevator", 0.3, 90.0, 10.0, new Color8Bit(0, 0, 255)));
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("SuperStructure/Elevator", m_inputs);

    runElevatorSetpoint(
        m_elevatorLevel != null ? m_elevatorLevel.setpointMeters : getPositionMeters());

    m_elevatorRoot.setPosition(0.13, 0.15 + m_inputs.elevatorPositionMeters);
    Logger.recordOutput("SuperStructure/Elevator/Mechanism2d", m_elevatorMech);

    // Update alerts
    motor1DisconnectedAlert.set(!m_inputs.elevatorM1Connected);
    motor2DisconnectedAlert.set(!m_inputs.elevatorM2Connected);
  }

  public void runElevatorOpenLoop(double outputVolts) {
    // TODO MUST match the real implementation!
    if (outputVolts > 0) {
      m_io.setVoltage(isWithinUpperBound(getPositionMeters()) ? outputVolts : 0.0);
    } else if (outputVolts < 0) {
      m_io.setVoltage(isWithinLowerBound(getPositionMeters()) ? outputVolts : 0.0);
    } else {
      m_io.setVoltage(outputVolts);
    }
  }

  public void runElevatorSetpoint(double setpointMeters) {
    if (m_goal.position != setpointMeters) {
      m_goal = new TrapezoidProfile.State(setpointMeters, 0.0);
    }
    TrapezoidProfile.State current = getCurrentState();
    TrapezoidProfile.State setpoint = m_elevatorProfile.calculate(0.02, current, m_goal);
    m_io.setVoltage(
        m_elevatorFeedforward.calculate(setpoint.velocity)
            + m_elevatorController.calculate(current.position, setpoint.position));
  }

  public void runCharacterization(double outputVolts) {
    runElevatorOpenLoop(outputVolts);
  }

  public void stop() {
    m_io.setVoltage(0.0);
  }

  private boolean isWithinUpperBound(double positionMeters) {
    return positionMeters < kElevatorUpperBoundMeters;
  }

  private boolean isWithinLowerBound(double positionMeters) {
    return positionMeters > kElevatorLowerBoundMeters;
  }

  public double getPositionMeters() {
    return m_inputs.elevatorPositionMeters;
  }

  public double getVelocityMetersPerSec() {
    return m_inputs.elevatorVelocityMetersPerSec;
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(getPositionMeters(), getVelocityMetersPerSec());
  }

  public void setLevel(ElevatorLevel elevatorLevel) {
    m_elevatorLevel = elevatorLevel;
  }

  public ElevatorLevel getLevel() {
    return m_elevatorLevel;
  }

  public boolean isSetpointAchieved() {
    return m_elevatorController.atSetpoint();
  }
}
