package frc.robot.subsystems.SuperStructure.Arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;

public class Arm {
  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert;

  private TrapezoidProfile m_armProfile =
      new TrapezoidProfile(new Constraints(kArmMaxVelocity, kArmMaxAcceleration));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TunablePIDController m_armController;
  private ArmFeedforward m_armFeedforward;
  private SysIdRoutine m_sysIdRoutine;

  private ArmGoals m_armGoal = ArmGoals.STOW;

  public Arm(ArmIO io) {
    m_io = io;

    switch (Constants.currentMode) {
      case REAL:
        m_armController =
            new TunablePIDController(armKp, 0.0, armKd, kArmErrorToleranceRads, "ArmPID", true);
        m_armFeedforward = new ArmFeedforward(armKs, armKg, armKv);
        break;
      case SIM:
        m_armController =
            new TunablePIDController(
                armSimKp, 0.0, armSimKd, kArmErrorToleranceRads, "ArmSimPID", true);
        m_armFeedforward = new ArmFeedforward(0.0, armSimKg, armSimKv);
        break;
      case REPLAY:
        m_armController =
            new TunablePIDController(
                armSimKp, 0.0, armSimKd, kArmErrorToleranceRads, "ArmSimPID", true);
        m_armFeedforward = new ArmFeedforward(0.0, armSimKg, armSimKv);
        break;
      default:
        m_armController = new TunablePIDController(0.0, 0.0, 0.0, 0.0, "", false);
        m_armFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
        break;
    }

    motorDisconnectedAlert = new Alert("Arm disconnected", AlertType.kError);
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("SuperStructure/Arm", m_inputs);

    // TODO comment this out for SysId
    runArmSetpoint(m_armGoal != null ? m_armGoal.setpointRadians : getPositionRadians());

    // Update alerts
    motorDisconnectedAlert.set(!m_inputs.armConnected);
  }

  public void runArmOpenLoop(double outputVolts) {
    // TODO MUST match the real implementation!
    if (outputVolts > 0) {
      m_io.setVoltage(isWithinMaximum(getPositionRadians()) ? outputVolts : 0.0);
    } else if (outputVolts < 0) {
      m_io.setVoltage(isWithinMinimum(getPositionRadians()) ? outputVolts : 0.0);
    } else {
      m_io.setVoltage(outputVolts);
    }
  }

  public void runArmSetpoint(double setpointRadians) {
    if (m_goal.position != setpointRadians) {
      m_goal = new TrapezoidProfile.State(setpointRadians, 0.0);
    }
    TrapezoidProfile.State current = getCurrentState();
    TrapezoidProfile.State setpoint = m_armProfile.calculate(0.02, current, m_goal);
    m_io.setVoltage(
        m_armFeedforward.calculate(setpoint.position, setpoint.velocity)
            + m_armController.calculate(current.position, setpoint.position));
  }

  // TODO May adjust limits to avoid damaging the mechanism
  public void runCharacterization(double outputVolts) {
    runArmOpenLoop(outputVolts);
  }

  public void stop() {
    m_io.setVoltage(0.0);
  }

  private boolean isWithinMaximum(double positionRadians) {
    return positionRadians < kArmMaxAngleRads;
  }

  private boolean isWithinMinimum(double positionRadians) {
    return positionRadians > kArmMinAngleRads;
  }

  public double getPositionRadians() {
    // return m_inputs.absolutePositionRads;
    return m_inputs.positionRads;
  }

  public double getVelocityRadPerSec() {
    return m_inputs.armVelocityRadPerSec;
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(getPositionRadians(), getVelocityRadPerSec());
  }

  public void setGoal(ArmGoals armGoal) {
    m_armGoal = armGoal;
  }

  public ArmGoals getGoal() {
    return m_armGoal;
  }

  public boolean isSetpointAchieved() {
    return Math.abs(m_goal.position - getPositionRadians()) < kArmErrorToleranceRads;
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
                    Logger.recordOutput("SuperStructure/Arm/ArmSysIDState", state.toString())),
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
