package frc.robot.subsystems.SuperStructure.Elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO m_io;
  private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();

  private final Alert motor1DisconnectedAlert;
  private final Alert motor2DisconnectedAlert;

  private TrapezoidProfile m_elevatorProfile =
      new TrapezoidProfile(new Constraints(kElevatorMaxVelocity, kElevatorMaxAcceleration));
  private TunablePIDController m_pidController;
  private ElevatorFeedforward m_feedforwardPID;
  private SysIdRoutine m_sysIdRoutine;

  private LinearSystem<N2, N1, N2> m_plant;
  private LinearQuadraticRegulator<N2, N1, N2> m_regulator;
  private LinearPlantInversionFeedforward<N2, N1, N2> m_feedForwardLQR;

  private boolean m_isSetpointAchievedInvalid = false;

  private TrapezoidProfile.State m_goalState =
      new TrapezoidProfile.State(kElevatorStartingPositionMeters, 0.0);
  private TrapezoidProfile.State m_prevSetpoint = m_goalState;
  private boolean m_runClosedLoop = true;

  public Elevator(ElevatorIO io) {
    m_io = io;

    // for state space
    m_plant = LinearSystemId.identifyPositionSystem(kElevatorKv, kElevatorKa);
    m_regulator =
        new LinearQuadraticRegulator<>(
            m_plant, VecBuilder.fill(0.1, 0.15), VecBuilder.fill(12.0), 0.02);
    if (Constants.kCurrentMode == Mode.REAL) {
      m_regulator.latencyCompensate(m_plant, 0.02, kElevatorLatencyCompensationMs);
    }
    m_feedForwardLQR = new LinearPlantInversionFeedforward<>(m_plant, 0.02);

    // KalmanFilter<N2, N1, N2> m_observer =
    //     new KalmanFilter<>(
    //         Nat.N2(),
    //         Nat.N2(),
    //         m_plant,
    //         VecBuilder.fill(3.0, 3.0),
    //         VecBuilder.fill(0.01, 0.01),
    //         0.02);

    // LinearSystemLoop<N2, N1, N2> m_loop = new LinearSystemLoop<>(m_plant, m_regulator,
    // m_observer, 12.0, 0.02);

    // for PID + FF
    m_pidController =
        new TunablePIDController(
            kElevatorKp, 0.0, kElevatorKd, kElevatorErrorToleranceMeters, "ElevatorPID", true);
    m_feedforwardPID = new ElevatorFeedforward(kElevatorKs, kElevatorKg, kElevatorKv, kElevatorKa);

    motor1DisconnectedAlert = new Alert("Elevator motor1 disconnected", AlertType.kError);
    motor2DisconnectedAlert = new Alert("Elevator motor2 disconnected", AlertType.kError);

    SmartDashboard.putBoolean("ElevatorStateSpace", true);
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("SuperStructure/Elevator", m_inputs);

    if (m_runClosedLoop) runClosedLoopControl();

    // Update alerts
    motor1DisconnectedAlert.set(!m_inputs.motor1Connected);
    motor2DisconnectedAlert.set(!m_inputs.motor2Connected);

    m_isSetpointAchievedInvalid = false;
  }

  public void runElevatorOpenLoop(double outputVolts) {
    // disable closed loop when running open loop
    m_runClosedLoop = false;
    if (outputVolts > 0) {
      m_io.setVoltage(isWithinMaximum(getPositionMeters()) ? outputVolts : 0.0);
    } else if (outputVolts < 0) {
      m_io.setVoltage(isWithinMinimum(getPositionMeters()) ? outputVolts : 0.0);
    } else {
      m_io.setVoltage(outputVolts);
    }
  }

  public void runClosedLoopControl() {
    TrapezoidProfile.State current = getCurrentState();
    TrapezoidProfile.State setpoint =
        m_elevatorProfile.calculate(0.02, m_prevSetpoint, m_goalState);
    m_prevSetpoint = setpoint;
    Logger.recordOutput("SuperStructure/Elevator/SetpointPosition", setpoint.position);
    Logger.recordOutput("SuperStructure/Elevator/SetpointVelocity", setpoint.velocity);

    if (SmartDashboard.getBoolean("ElevatorStateSpace", true)) {
      Vector<N2> nextR = VecBuilder.fill(setpoint.position, setpoint.velocity);
      double voltage =
          m_regulator
              .calculate(VecBuilder.fill(getPositionMeters(), getVelocityMetersPerSec()), nextR)
              .plus(
                  m_feedForwardLQR
                      .calculate(nextR)
                      .plus(kElevatorKs * Math.signum(setpoint.velocity) + kElevatorKg))
              .get(0, 0);
      m_io.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    } else {
      m_io.setVoltage(
          m_feedforwardPID.calculate(setpoint.velocity)
              + m_pidController.calculate(current.position, setpoint.position));
    }
  }

  public void runCharacterization(double outputVolts) {
    // disable closed loop when running characterization
    m_runClosedLoop = false;
    m_io.setVoltage(outputVolts);
  }

  public void stop() {
    m_io.setVoltage(0.0);
  }

  /**
   * Stops the elevator and sets the goalPosition to the current position. This will hold the
   * elevator in place until a new goal is set.
   */
  public void stopAndHold() {
    m_io.setVoltage(0.0);

    // set setpoint (this will enable closed loop)
    setGoalPosition(getPositionMeters());
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

  public void setBrakeMode(boolean brakeModeEnabled) {
    m_io.setBrakeMode(brakeModeEnabled);
    stop();
    m_runClosedLoop = false;
  }

  @AutoLogOutput(key = "SuperStructure/Elevator/AdjustmentCoefficient")
  public double getAdjustmentCoefficient() {
    return Math.abs(getPositionMeters() / kElevatorMaxHeightMeters);
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(getPositionMeters(), getVelocityMetersPerSec());
  }

  public void setGoalPosition(double positionMeters) {
    setGoalState(new TrapezoidProfile.State(positionMeters, 0.0));
  }

  public void setGoalState(TrapezoidProfile.State goal) {
    m_goalState = goal;
    // enable closed loop when a goal is set
    m_runClosedLoop = true;
  }

  public double getGoalPosition() {
    return getGoalState().position;
  }

  public TrapezoidProfile.State getGoalState() {
    return m_goalState;
  }

  @AutoLogOutput(key = "SuperStructure/Elevator/SetpointAchieved")
  public boolean isSetpointAchieved() {
    return (Math.abs(m_goalState.position - getPositionMeters()) < kElevatorErrorToleranceMeters)
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
