package frc.robot.subsystems.SuperStructure.Arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ArmFeedforward;
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

public class Arm {
  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();

  private final Alert motorDisconnectedAlert;

  private TrapezoidProfile m_armProfile =
      new TrapezoidProfile(new Constraints(kArmMaxVelocity, kArmMaxAcceleration));
  private TunablePIDController m_pidController;
  private ArmFeedforward m_feedforwardPID;
  private SysIdRoutine m_sysIdRoutine;

  private LinearSystem<N2, N1, N2> m_plant;
  private LinearQuadraticRegulator<N2, N1, N2> m_regulator;
  private LinearPlantInversionFeedforward<N2, N1, N2> m_feedForwardLQR;

  private boolean m_isSetpointAchievedInvalid = false;

  private TrapezoidProfile.State m_goalState =
      new TrapezoidProfile.State(kArmStartingPositionRads, 0.0);
  private TrapezoidProfile.State m_prevSetpoint = m_goalState;
  private boolean m_runClosedLoop = true;

  public Arm(ArmIO io) {
    m_io = io;

    // for state space
    m_plant = LinearSystemId.identifyPositionSystem(kArmKv, kArmKa);
    m_regulator =
        new LinearQuadraticRegulator<>(
            m_plant, VecBuilder.fill(0.1, 10.0), VecBuilder.fill(12.0), 0.02);
    if (Constants.kCurrentMode == Mode.REAL) {
      m_regulator.latencyCompensate(m_plant, 0.02, kArmLatencyCompensationMs);
    }
    m_feedForwardLQR = new LinearPlantInversionFeedforward<>(m_plant, 0.02);

    // for PID + FF
    m_pidController =
        new TunablePIDController(kArmKp, 0.0, kArmKd, kArmErrorToleranceRads, "ArmPID", true);
    m_feedforwardPID = new ArmFeedforward(kArmKs, kArmKg, kArmKv, kArmKa);

    motorDisconnectedAlert = new Alert("Arm motor disconnected", AlertType.kError);

    SmartDashboard.putBoolean("ArmStateSpace", true);
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("SuperStructure/Arm", m_inputs);

    if (m_runClosedLoop) runClosedLoopControl();

    // Update alerts
    motorDisconnectedAlert.set(!m_inputs.motorConnected);

    m_isSetpointAchievedInvalid = false;
  }

  public void runArmOpenLoop(double outputVolts) {
    // disable closed loop when running open loop
    m_runClosedLoop = false;
    if (outputVolts > 0) {
      m_io.setVoltage(isWithinMaximum(getPositionRads()) ? outputVolts : 0.0);
    } else if (outputVolts < 0) {
      m_io.setVoltage(isWithinMinimum(getPositionRads()) ? outputVolts : 0.0);
    } else {
      m_io.setVoltage(outputVolts);
    }
  }

  public void runClosedLoopControl() {
    TrapezoidProfile.State current = getCurrentState();
    TrapezoidProfile.State setpoint = m_armProfile.calculate(0.02, m_prevSetpoint, m_goalState);
    m_prevSetpoint = setpoint;
    Logger.recordOutput("SuperStructure/Arm/SetpointPosition", setpoint.position);
    Logger.recordOutput("SuperStructure/Arm/SetpointVelocity", setpoint.velocity);

    if (SmartDashboard.getBoolean("ArmStateSpace", true)) {
      // state space
      Vector<N2> nextR = VecBuilder.fill(setpoint.position, setpoint.velocity);
      double voltage =
          m_regulator
              .calculate(VecBuilder.fill(getPositionRads(), getVelocityRadsPerSec()), nextR)
              .plus(
                  m_feedForwardLQR
                      .calculate(nextR)
                      .plus(
                          kArmKs * Math.signum(setpoint.velocity)
                              + kArmKg * Math.cos(setpoint.position)))
              .get(0, 0);
      m_io.setVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
    } else {
      // PID + FF
      m_io.setVoltage(
          m_feedforwardPID.calculate(setpoint.position, setpoint.velocity)
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
   * Stops the arm and sets the goalPosition to the current position. This will hold the arm in
   * place until a new goal is set.
   */
  public void stopAndHold() {
    m_io.setVoltage(0.0);

    // set setpoint (this will enable closed loop)
    setGoalPosition(getPositionRads());
  }

  private boolean isWithinMaximum(double positionRads) {
    return positionRads < kArmMaxAngleRads;
  }

  private boolean isWithinMinimum(double positionRads) {
    return positionRads > kArmMaxAngleRads;
  }

  public double getPositionRads() {
    return m_inputs.positionRads;
  }

  public double getVelocityRadsPerSec() {
    return m_inputs.velocityRadsPerSec;
  }

  public void setBrakeMode(boolean brakeModeEnabled) {
    m_io.setBrakeMode(brakeModeEnabled);
    stop();
    m_runClosedLoop = false;
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(getPositionRads(), getVelocityRadsPerSec());
  }

  public void setGoalPosition(double positionRads) {
    setGoalState(new TrapezoidProfile.State(positionRads, 0.0));
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

  @AutoLogOutput(key = "SuperStructure/Arm/SetpointAchieved")
  public boolean isSetpointAchieved() {
    return (Math.abs(m_goalState.position - getPositionRads()) < kArmErrorToleranceRads)
        && !m_isSetpointAchievedInvalid;
  }

  // Configure SysId
  public void sysIdSetup(SuperStructure superStructure) {
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volt.of(0.5).per(Second),
                Volt.of(3.5),
                null,
                (state) -> Logger.recordOutput("SuperStructure/Arm/SysIDState", state.toString())),
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
