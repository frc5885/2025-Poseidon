package frc.robot.subsystems.SuperStructure.Wrist;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.*;

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
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;

public class Wrist {
  private final WristIO m_io;
  private final WristIOInputsAutoLogged m_inputs = new WristIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert;

  private TrapezoidProfile m_wristProfile =
      new TrapezoidProfile(new Constraints(kWristMaxVelocity, kWristMaxAcceleration));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TunablePIDController m_wristController;
  private ArmFeedforward m_wristFeedforward;
  private SysIdRoutine m_sysIdRoutine;

  public Wrist(WristIO io) {
    m_io = io;

    switch (Constants.kCurrentMode) {
      case REAL:
        m_wristController =
            new TunablePIDController(
                kWristKp, 0.0, kWristKd, kWristErrorToleranceRads, "WristPID", true);
        m_wristFeedforward = new ArmFeedforward(kWristKs, kWristKg, kWristKv);
        break;
      case SIM:
        m_wristController =
            new TunablePIDController(
                kWristSimKp, 0.0, kWristSimKd, kWristErrorToleranceRads, "WristSimPID", true);
        m_wristFeedforward = new ArmFeedforward(0.0, kWristSimKg, kWristSimKv);
        break;
      case REPLAY:
        m_wristController =
            new TunablePIDController(
                kWristSimKp, 0.0, kWristSimKd, kWristErrorToleranceRads, "WristSimPID", true);
        m_wristFeedforward = new ArmFeedforward(0.0, kWristSimKg, kWristSimKv);
        break;
      default:
        m_wristController = new TunablePIDController(0.0, 0.0, 0.0, 0.0, "", false);
        m_wristFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
        break;
    }

    motorDisconnectedAlert = new Alert("Wrist disconnected", AlertType.kError);
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("SuperStructure/Wrist", m_inputs);

    // TODO comment this out for SysId
    // runArmSetpoint(m_armGoal != null ? m_armGoal.setpointRadians : getPositionRadians());

    // Update alerts
    motorDisconnectedAlert.set(!m_inputs.wristConnected);
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

  // public void runArmSetpoint(double setpointRadians) {
  //     if (m_goal.position != setpointRadians) {
  //     m_goal = new TrapezoidProfile.State(setpointRadians, 0.0);
  //     }
  //     TrapezoidProfile.State current = getCurrentState();
  //     TrapezoidProfile.State setpoint = m_armProfile.calculate(0.02, current, m_goal);
  //     m_io.setVoltage(
  //         m_armFeedforward.calculate(setpoint.position, setpoint.velocity)
  //             + m_armController.calculate(current.position, setpoint.position));
  // }

  // TODO May adjust limits to avoid damaging the mechanism
  public void runCharacterization(double outputVolts) {
    m_io.setVoltage(outputVolts);
  }

  public void stop() {
    m_io.setVoltage(0.0);
  }

  private boolean isWithinMaximum(double positionRadians) {
    return positionRadians < kWristMaxAngleRads;
  }

  private boolean isWithinMinimum(double positionRadians) {
    return positionRadians > kWristMinAngleRads;
  }

  public double getPositionRadians() {
    return m_inputs.positionRads;
  }

  public double getRealWorldPositionRads() {
    return m_inputs.realWorldPositionRads;
  }

  public double getVelocityRadPerSec() {
    return m_inputs.wristVelocityRadPerSec;
  }

  public TrapezoidProfile.State getCurrentState() {
    return new TrapezoidProfile.State(getPositionRadians(), getVelocityRadPerSec());
  }

  // public void setGoal(ArmGoals armGoal) {
  //     m_armGoal = armGoal;
  // }

  // public ArmGoals getGoal() {
  //     return m_armGoal;
  // }

  // public boolean isSetpointAchieved() {
  //     return Math.abs(m_goal.position - getPositionRadians()) < kArmErrorToleranceRads;
  // }

  // Configure SysId
  public void sysIdSetup(SuperStructure superStructure) {
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) ->
                    Logger.recordOutput("SuperStructure/Wrist/WristSysIDState", state.toString())),
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
