package frc.robot.subsystems.SuperStructure.Arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.kWristStartingPositionRadians;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.util.TunableDouble;
import frc.robot.util.TunablePIDController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm {
  private final ArmIO m_io;
  private final ArmIOInputsAutoLogged m_inputs = new ArmIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert;
  private final BooleanSupplier m_disablePIDs;

  // Track previous disabled state to detect rising edge
  private boolean m_wasDisabled = false;

  private TrapezoidProfile m_armProfile =
      new TrapezoidProfile(new Constraints(kArmMaxVelocity, kArmMaxAcceleration));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TunablePIDController m_armController;
  private ArmFeedforward m_armFeedforward;
  private SysIdRoutine m_sysIdRoutine;

  private LinearSystem<N2, N1, N2> m_plant;
  private LinearQuadraticRegulator<N2, N1, N2> m_regulator;
  private DoubleSupplier m_setpoint =
      TunableDouble.register("Arm/OverrideSetpoint", kArmStartingPositionRadians);

  private ArmGoals m_armGoal = ArmGoals.STOW;
  private boolean m_isSetpointAchievedInvalid = false;
  private boolean m_pidOff = false;

  private DoubleSupplier m_wristAngleRadSupplier = () -> kWristStartingPositionRadians;

  public Arm(ArmIO io, BooleanSupplier disablePIDs) {
    m_io = io;
    m_disablePIDs = disablePIDs;

    m_plant =
        LinearSystemId.createSingleJointedArmSystem(
            DCMotor.getNeo550(1), kArmStowedMOI_kgm2, kArmMotorReduction);
    m_regulator =
        new LinearQuadraticRegulator<>(
            m_plant, VecBuilder.fill(0.1E-10, 9.9E10), VecBuilder.fill(9.9E10), 0.02);

    switch (Constants.kCurrentMode) {
      case REAL:
        m_armController =
            new TunablePIDController(kArmKp, 0.0, kArmKd, kArmErrorToleranceRads, "ArmPID", true);
        m_armFeedforward = new ArmFeedforward(kArmKs, kArmStowedKg, kArmKv);
        break;
      case SIM:
        m_armController =
            new TunablePIDController(
                kArmSimKp, 0.0, kArmSimKd, kArmErrorToleranceRads, "ArmSimPID", true);
        m_armFeedforward = new ArmFeedforward(0.0, kArmSimKg, kArmSimKv);
        break;
      case REPLAY:
        m_armController =
            new TunablePIDController(
                kArmSimKp, 0.0, kArmSimKd, kArmErrorToleranceRads, "ArmSimPID", true);
        m_armFeedforward = new ArmFeedforward(0.0, kArmSimKg, kArmSimKv);
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

    if (!m_pidOff) {
      boolean isDisabled = m_disablePIDs.getAsBoolean();
      if (!isDisabled) {
        runArmSetpoint(
            m_armGoal != null
                ? Units.degreesToRadians(m_armGoal.setpointDegrees.getAsDouble())
                : getPositionRadians());
      } else if (!m_wasDisabled) {
        // Only call stop() on the rising edge of m_disablePIDs
        stop();
      }
      m_wasDisabled = isDisabled;

      m_io.setVoltage(
          m_regulator
              .calculate(
                  VecBuilder.fill(getPositionRadians(), getVelocityRadPerSec()),
                  VecBuilder.fill(Units.degreesToRadians(m_setpoint.getAsDouble()), 0.0))
              .get(0, 0));
    }
    // Update alerts
    motorDisconnectedAlert.set(!m_inputs.armConnected);
    m_isSetpointAchievedInvalid = false;
  }

  public void runArmOpenLoop(double outputVolts) {
    m_pidOff = true;
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

    double compensatedKg = kArmStowedKg;
    if (Robot.isReal()) {
      compensatedKg =
          kArmOutKg + (kArmOutKg - kArmStowedKg) * Math.cos(m_wristAngleRadSupplier.getAsDouble());
    }
    m_armFeedforward.setKg(compensatedKg);
    Logger.recordOutput("SuperStructure/Arm/ArmKg", compensatedKg);

    m_io.setVoltage(
        m_armFeedforward.calculate(setpoint.position, setpoint.velocity)
            + m_armController.calculate(current.position, setpoint.position));
  }

  public void runCharacterization(double outputVolts) {
    m_io.setVoltage(outputVolts);
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

  public void setWristAngleRadSupplier(DoubleSupplier wristAngleRadSupplier) {
    m_wristAngleRadSupplier = wristAngleRadSupplier;
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
    if (m_armGoal == armGoal) {
      return;
    }
    m_isSetpointAchievedInvalid = true;
    m_armGoal = armGoal;
  }

  public ArmGoals getGoal() {
    return m_armGoal;
  }

  public double getSetpointRadians() {
    return m_armController.getSetpoint();
  }

  @AutoLogOutput(key = "SuperStructure/Arm/SetpointAchieved")
  public boolean isSetpointAchieved() {
    return (Math.abs(m_goal.position - getPositionRadians()) < kArmErrorToleranceRads)
        && !m_isSetpointAchievedInvalid;
  }

  // Configure SysId
  public void sysIdSetup(SuperStructure superStructure) {
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.7).per(Second),
                Volts.of(5),
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

  public void setPIDOff(boolean flag) {
    m_pidOff = flag;
  }
}
