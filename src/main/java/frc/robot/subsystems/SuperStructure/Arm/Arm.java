package frc.robot.subsystems.SuperStructure.Arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.kWristStartingPositionRadians;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.ArmGoals;
import frc.robot.util.TunableDouble;
import frc.robot.util.TunableFeedForward;
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

  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TunablePIDController m_armController;
  private TunableFeedForward m_armFeedforward;
  private SysIdRoutine m_sysIdRoutine;

  private ArmGoals m_armGoal = ArmGoals.STOW;
  private boolean m_isSetpointAchievedInvalid = false;

  private DoubleSupplier m_wristAngleRadSupplier = () -> kWristStartingPositionRadians;

  private DoubleSupplier kG = TunableDouble.register("Arm/kG", 0.5);

  private TalonFX m_arm;
  private DutyCycleEncoder m_encoder;
  private double m_lastPosition;
  private double m_setPointRadians = 2.94;
  private final double kMaxVelocity = 1.15;
  private final double kMaxAccel = 11.0;
  private final double kVelDeviation = 0.040337;

  private TrapezoidProfile m_armProfile =
      new TrapezoidProfile(new Constraints(kMaxVelocity, kMaxAccel));

  public Arm(ArmIO io, BooleanSupplier disablePIDs) {
    m_arm = new TalonFX(60);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Feedback.SensorToMechanismRatio = 76.57;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) {
      status = m_arm.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    m_encoder = new DutyCycleEncoder(1);
    m_encoder.setDutyCycleRange(kArmEncoderMin, kArmEncoderMax);
    m_arm.setPosition(2.94);

    m_io = io;
    m_disablePIDs = disablePIDs;

    switch (Constants.kCurrentMode) {
      case REAL:
        m_armController =
            new TunablePIDController(10.0, 0.0, 0.5, kArmErrorToleranceRads, "ArmPID", true);
        m_armFeedforward =
            new TunableFeedForward(0.36392, 0.29185, 7.6337, 0.0, "ArmFeedForward", true);
        // kA = 0.17852
        break;
      case SIM:
        m_armController =
            new TunablePIDController(
                kArmSimKp, 0.0, kArmSimKd, kArmErrorToleranceRads, "ArmSimPID", true);
        m_armFeedforward =
            new TunableFeedForward(0.0, kArmSimKg, kArmSimKv, 0.0, "ArmSimFeedForward", true);
        break;
      case REPLAY:
        m_armController =
            new TunablePIDController(
                kArmSimKp, 0.0, kArmSimKd, kArmErrorToleranceRads, "ArmSimPID", true);
        m_armFeedforward =
            new TunableFeedForward(0.0, kArmSimKg, kArmSimKv, 0.0, "ArmSimFeedForward", true);
        break;
      default:
        m_armController = new TunablePIDController(0.0, 0.0, 0.0, 0.0, "", false);
        m_armFeedforward = new TunableFeedForward(0.0, 0.0, 0.0, 0.0, "", false);
        break;
    }

    motorDisconnectedAlert = new Alert("Arm disconnected", AlertType.kError);
  }

  private double getArmPositionFalcon() {
    return m_arm.getPosition().getValueAsDouble();
  }

  private double getArmVelocity() {
    return m_arm.getVelocity().getValueAsDouble();
  }

  private double getArmVoltage() {
    return m_arm.getMotorVoltage().getValueAsDouble();
  }

  public void setArmOpenLoop(double inputVolt) {
    m_arm.setVoltage(inputVolt);
  }

  public void setPose(double setpoint) {
    m_setPointRadians = setpoint;
  }

  public void periodic() {
    Logger.recordOutput("TestArm/Position", getArmPositionFalcon());
    Logger.recordOutput("TestArm/Velocity", getArmVelocity());
    Logger.recordOutput("TestArm/InputVolts", getArmVoltage());
    Logger.recordOutput("TestArm/Goal", m_setPointRadians);

    if (m_goal.position != m_setPointRadians) {
      m_goal = new TrapezoidProfile.State(m_setPointRadians, 0.0);
    }
    TrapezoidProfile.State current = getCurrentState();
    TrapezoidProfile.State setpoint = m_armProfile.calculate(0.02, current, m_goal);
    Logger.recordOutput("TestArm/Setpoint", setpoint.position);
    m_arm.setVoltage(
        m_armFeedforward.calculate(setpoint.position, setpoint.velocity)
            + m_armController.calculate(current.position, m_goal.position));

    // m_io.updateInputs(m_inputs);
    // Logger.processInputs("SuperStructure/Arm", m_inputs);

    // boolean isDisabled = m_disablePIDs.getAsBoolean();
    // if (!isDisabled) {
    //   runArmSetpoint(
    //       m_armGoal != null
    //           ? Units.degreesToRadians(m_armGoal.setpointDegrees.getAsDouble())
    //           : getPositionRadians());
    // } else if (!m_wasDisabled) {
    //   // Only call stop() on the rising edge of m_disablePIDs
    //   stop();
    // }
    // m_wasDisabled = isDisabled;

    // // Update alerts
    // motorDisconnectedAlert.set(!m_inputs.armConnected);
    // m_isSetpointAchievedInvalid = false;
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

    double compensatedKg = kArmStowedKg;
    // if (Robot.isReal()) {
    //   compensatedKg =
    //       kArmOutKg + (kArmOutKg - kArmStowedKg) *
    // Math.cos(m_wristAngleRadSupplier.getAsDouble());
    // }
    m_armFeedforward.setKg(compensatedKg);
    Logger.recordOutput("SuperStructure/Arm/ArmKg", compensatedKg);

    m_io.setVoltage(
        // m_armFeedforward.calculate(setpoint.position, setpoint.velocity) +
        m_armController.calculate(current.position, m_goal.position)
            + kG.getAsDouble() * Math.cos(m_inputs.positionRads));
  }

  public void runCharacterization(double outputVolts) {
    // m_io.setVoltage(outputVolts);
    m_arm.setVoltage(outputVolts);
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
    return new TrapezoidProfile.State(getArmPositionFalcon(), getArmVelocity());
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
}
