package frc.robot.subsystems.SuperStructure.Elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.*;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.ElevatorLevel;
import frc.robot.util.TunablePIDController;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO m_io;
  private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();
  private final BooleanSupplier m_disablePIDs;

  private final Alert motor1DisconnectedAlert;
  private final Alert motor2DisconnectedAlert;

  private TrapezoidProfile m_elevatorProfile =
      new TrapezoidProfile(new Constraints(kElevatorMaxVelocity, kElevatorMaxAcceleration));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TunablePIDController m_elevatorController;
  private ElevatorFeedforward m_elevatorFeedforward;
  private SysIdRoutine m_sysIdRoutine;

  private ElevatorLevel m_elevatorGoal = ElevatorLevel.STOW;
  private boolean m_isSetpointAchievedInvalid = false;

  public Elevator(ElevatorIO io, BooleanSupplier disablePIDs) {
    m_io = io;
    m_disablePIDs = disablePIDs;

    switch (Constants.kCurrentMode) {
      case REAL:
        m_elevatorController =
            new TunablePIDController(
                kElevatorKp, 0.0, kElevatorKd, kElevatorErrorToleranceMeters, "ElevatorPID", true);
        m_elevatorFeedforward = new ElevatorFeedforward(kElevatorKs, kElevatorKg, kElevatorKv);
        break;
      case SIM:
        m_elevatorController =
            new TunablePIDController(
                kElevatorSimKp,
                0.0,
                kElevatorSimKd,
                kElevatorErrorToleranceMeters,
                "ElevatorSimPID",
                true);
        m_elevatorFeedforward = new ElevatorFeedforward(0.0, kElevatorSimKg, kElevatorSimKv);
        break;
      case REPLAY:
        m_elevatorController =
            new TunablePIDController(
                kElevatorSimKp,
                0.0,
                kElevatorSimKd,
                kElevatorErrorToleranceMeters,
                "ElevatorReplayPID",
                true);
        m_elevatorFeedforward = new ElevatorFeedforward(0.0, kElevatorSimKg, kElevatorSimKv);
        break;
      default:
        m_elevatorController = new TunablePIDController(0.0, 0.0, 0.0, 0.0, "", false);
        m_elevatorFeedforward = new ElevatorFeedforward(0.0, 0.0, 0.0);
        break;
    }

    motor1DisconnectedAlert = new Alert("Elevator motor1 disconnected", AlertType.kError);
    motor2DisconnectedAlert = new Alert("Elevator motor2 disconnected", AlertType.kError);
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("SuperStructure/Elevator", m_inputs);

    if (!m_disablePIDs.getAsBoolean()) {
      runElevatorSetpoint(
          m_elevatorGoal != null
              ? m_elevatorGoal.setpointMeters.getAsDouble()
              : getPositionMeters());
    } else {
      stop();
    }

    // Update alerts
    motor1DisconnectedAlert.set(!m_inputs.motor1Connected);
    motor2DisconnectedAlert.set(!m_inputs.motor2Connected);

    m_isSetpointAchievedInvalid = false;
  }

  public void runElevatorOpenLoop(double outputVolts) {
    // TODO MUST match the real implementation!
    if (outputVolts > 0) {
      m_io.setVoltage(isWithinMaximum(getPositionMeters()) ? outputVolts : 0.0);
    } else if (outputVolts < 0) {
      m_io.setVoltage(isWithinMinimum(getPositionMeters()) ? outputVolts : 0.0);
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
    m_io.setVoltage(outputVolts);
  }

  public void stop() {
    m_io.setVoltage(0.0);
  }

  private boolean isWithinMaximum(double positionMeters) {
    return positionMeters < kElevatorMaxHeightMeters;
  }

  private boolean isWithinMinimum(double positionMeters) {
    return positionMeters > kElevatorMinHeightMeters;
  }

  public double getPositionMeters() {
    return m_inputs.positionMeters;
  }

  public double getVelocityMetersPerSec() {
    return m_inputs.velocityMetersPerSec;
  }

  @AutoLogOutput(key = "SuperStructure/Elevator/AdjustmentCoefficient")
  public double getAdjustmentCoefficient() {
    return Math.abs(getPositionMeters() / kElevatorMaxHeightMeters);
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(getPositionMeters(), getVelocityMetersPerSec());
  }

  public void setGoal(ElevatorLevel elevatorGoal) {
    if (m_elevatorGoal == elevatorGoal) {
      return;
    }
    m_isSetpointAchievedInvalid = true;
    m_elevatorGoal = elevatorGoal;
  }

  public ElevatorLevel getGoal() {
    return m_elevatorGoal;
  }

  public double getSetpointRadians() {
    return m_elevatorController.getSetpoint();
  }

  @AutoLogOutput(key = "SuperStructure/Elevator/SetpointAchieved")
  public boolean isSetpointAchieved() {
    return (Math.abs(m_goal.position - getPositionMeters()) < kElevatorErrorToleranceMeters)
        && !m_isSetpointAchievedInvalid;
  }

  // Configure SysId
  public void sysIdSetup(SuperStructure superStructure) {
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput("SuperStructure/ElevatorSysIDState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, superStructure));
  }

  public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(m_sysIdRoutine.quasistatic(direction));
  }

  public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
    return Commands.run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(m_sysIdRoutine.dynamic(direction));
  }
}
