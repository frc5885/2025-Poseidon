package frc.robot.subsystems.SuperStructure.Wrist;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.kArmMaxVelocity;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.kArmStartingPositionRadians;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.WristGoals;
import frc.robot.util.TunablePIDController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist {
  private final WristIO m_io;
  private final WristIOInputsAutoLogged m_inputs = new WristIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert;
  private final BooleanSupplier m_disablePIDs;

  // Track previous disabled state to detect rising edge
  private boolean m_wasDisabled = false;

  private TrapezoidProfile m_wristProfile =
      new TrapezoidProfile(new Constraints(kWristMaxVelocity, kWristMaxAcceleration));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TunablePIDController m_wristController;
  private ArmFeedforward m_wristFeedforward;
  private double m_wristBaseKV = 0.0;
  private double m_ffOffset = 0.0;
  private SysIdRoutine m_sysIdRoutine;

  private WristGoals m_wristGoal = WristGoals.STOW;
  private boolean m_isSetpointAchievedInvalid = false;

  private DoubleSupplier m_armAngleRadSupplier = () -> kArmStartingPositionRadians;
  private DoubleSupplier m_armAngularVelocitySupplier = () -> 0.0;

  // Add variables to track arm motion for acceleration calculation
  private double m_lastArmVelocity = 0.0;
  private double m_armAcceleration = 0.0;

  public Wrist(WristIO io, BooleanSupplier disablePIDs) {
    m_io = io;
    m_disablePIDs = disablePIDs;

    switch (Constants.kCurrentMode) {
      case REAL:
        m_wristController =
            new TunablePIDController(
                kWristKp, 0.0, kWristKd, kWristErrorToleranceRads, "WristPID", true);
        m_wristFeedforward = new ArmFeedforward(kWristKs, kWristKg, kWristKv);
        m_wristBaseKV = kWristKv;
        m_ffOffset = kWristCOGOffsetForFFRadians; // need this on real robot
        break;
      case SIM:
        m_wristController =
            new TunablePIDController(
                kWristSimKp, 0.0, kWristSimKd, kWristErrorToleranceRads, "WristSimPID", true);
        m_wristFeedforward = new ArmFeedforward(0.0, kWristSimKg, kWristSimKv);
        m_wristBaseKV = kWristSimKv;
        break;
      case REPLAY:
        m_wristController =
            new TunablePIDController(
                kWristSimKp, 0.0, kWristSimKd, kWristErrorToleranceRads, "WristSimPID", true);
        m_wristFeedforward = new ArmFeedforward(0.0, kWristSimKg, kWristSimKv);
        m_wristBaseKV = kWristSimKv;
        break;
      default:
        m_wristController = new TunablePIDController(0.0, 0.0, 0.0, 0.0, "", false);
        m_wristFeedforward = new ArmFeedforward(0.0, 0.0, 0.0);
        break;
    }

    motorDisconnectedAlert = new Alert("Wrist disconnected", AlertType.kError);
  }

  // set the supplier to pass the arm angle to the wrist
  // needs to be called right after creating the wrist subsystem
  public void setArmAngleSuppliers(
      DoubleSupplier armAngleRadSupplier, DoubleSupplier armAngularVelocitySupplier) {
    m_armAngleRadSupplier = armAngleRadSupplier;
    m_armAngularVelocitySupplier = armAngularVelocitySupplier;
  }

  public void periodic() {
    double currentArmVelocity = m_armAngularVelocitySupplier.getAsDouble();
    // Calculate arm acceleration (rad/s^2)
    m_armAcceleration = (currentArmVelocity - m_lastArmVelocity) / 0.02;
    m_lastArmVelocity = currentArmVelocity;

    m_io.updateInputs(m_inputs, m_armAngleRadSupplier);
    Logger.processInputs("SuperStructure/Wrist", m_inputs);

    boolean isDisabled = m_disablePIDs.getAsBoolean();
    if (!isDisabled) {
      runWristSetpoint(
          m_wristGoal != null
              ? Units.degreesToRadians(m_wristGoal.setpointDegrees.getAsDouble())
              : getRealWorldPositionRadians());
    } else if (!m_wasDisabled) {
      // Only call stop() on the rising edge of m_disablePIDs
      stop();
    }
    m_wasDisabled = isDisabled;

    // Update alerts
    motorDisconnectedAlert.set(!m_inputs.wristConnected);
    m_isSetpointAchievedInvalid = false;
  }

  public void runWristOpenLoop(double outputVolts) {
    // TODO MUST match the real implementation!
    if (outputVolts > 0) {
      m_io.setVoltage(isWithinMaximum(getPositionRadians()) ? outputVolts : 0.0);
    } else if (outputVolts < 0) {
      m_io.setVoltage(isWithinMinimum(getPositionRadians()) ? outputVolts : 0.0);
    } else {
      m_io.setVoltage(outputVolts);
    }
  }

  public void runWristSetpoint(double setpointRadians) {
    if (setpointRadians == Units.degreesToRadians(WristGoals.LOCK.setpointDegrees.getAsDouble())) {
      // LOCK
      stop();
      return;
    }

    if (m_goal.position != setpointRadians) {
      m_goal = new TrapezoidProfile.State(setpointRadians, 0.0);
    }

    TrapezoidProfile.State current = getCurrentState();
    TrapezoidProfile.State setpoint = m_wristProfile.calculate(0.02, current, m_goal);

    // Get arm motion values
    double armVelocity = m_armAngularVelocitySupplier.getAsDouble();
    double armVelocityAbs = Math.abs(armVelocity);
    double wristVelocity = m_inputs.wristVelocityRadPerSec;

    // Calculate compensations for arm movement
    double velocityCompensation = 0.0;
    double accelerationCompensation = 0.0;

    // Velocity-based compensation (improved from original)
    if (armVelocityAbs > 0.05) { // Lower threshold to respond to smaller arm movements
      // Determine if the wrist needs to work against or with arm motion
      // Negative sign when arm and wrist move in opposite directions
      double relativeDirection = -Math.signum(armVelocity * wristVelocity);

      // Scale compensation based on arm velocity as a percentage of max
      double velocityRatio = armVelocityAbs / kArmMaxVelocity;

      // More aggressive compensation (0.12 instead of 0.08)
      velocityCompensation = relativeDirection * 0.12 * velocityRatio * m_wristBaseKV;

      // Add specific compensation for holding position against arm motion
      if (Math.abs(wristVelocity) < 0.1 && Math.abs(setpoint.velocity) < 0.1) {
        // Extra compensation when trying to hold position during arm movement
        velocityCompensation += Math.signum(armVelocity) * 0.08 * velocityRatio * m_wristBaseKV;
      }
    }

    // Acceleration-based compensation
    double armAccelAbs = Math.abs(m_armAcceleration);
    if (armAccelAbs > 0.5) { // Only compensate for significant acceleration
      // Calculate how much the acceleration affects the wrist
      // For a rapid arm deceleration, the wrist will want to continue moving
      double accelScale = 0.05; // Tunable parameter
      accelerationCompensation =
          -Math.signum(m_armAcceleration)
              * accelScale
              * (armAccelAbs / (kWristMaxAcceleration / 2))
              * m_wristBaseKV;

      // Log the acceleration compensation
      Logger.recordOutput(
          "SuperStructure/Wrist/AccelerationCompensation", accelerationCompensation);
    }

    // Apply combined compensations
    double adjustedKV = m_wristBaseKV + velocityCompensation + accelerationCompensation;

    // Ensure kV doesn't go negative or get too large
    adjustedKV = Math.max(m_wristBaseKV * 0.5, Math.min(m_wristBaseKV * 2.0, adjustedKV));

    // Apply the compensation to the feedforward
    m_wristFeedforward.setKv(adjustedKV);

    // Log the adjusted kV value
    Logger.recordOutput("SuperStructure/Wrist/AdjustedKV", adjustedKV);
    Logger.recordOutput("SuperStructure/Wrist/ArmAcceleration", m_armAcceleration);

    // Calculate control outputs
    // Apply COG offset to position used for feedforward calculation only
    double ffPosition = setpoint.position + m_ffOffset;
    double ffVoltage = m_wristFeedforward.calculate(ffPosition, setpoint.velocity);
    double pidVoltage = m_wristController.calculate(current.position, setpoint.position);

    // Add a direct acceleration compensation term when arm is accelerating rapidly
    if (armAccelAbs > 2.0) {
      // Direct compensation voltage based on arm acceleration direction
      double directCompVoltage =
          -Math.signum(m_armAcceleration) * Math.min(0.5, armAccelAbs * 0.04); // Limit to 0.5V max
      ffVoltage += directCompVoltage;
      Logger.recordOutput("SuperStructure/Wrist/DirectAccelCompVoltage", directCompVoltage);
    }

    Logger.recordOutput("SuperStructure/Wrist/FFVoltage", ffVoltage);
    Logger.recordOutput("SuperStructure/Wrist/PIDVoltage", pidVoltage);
    Logger.recordOutput("SuperStructure/Wrist/GoalPosition", setpointRadians);
    Logger.recordOutput("SuperStructure/Wrist/SetpointPosition", setpoint.position);
    m_io.setVoltage(ffVoltage + pidVoltage);
  }

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

  public double getRealWorldPositionRadians() {
    return m_inputs.realWorldPositionRads;
  }

  public double getVelocityRadPerSec() {
    return m_inputs.wristVelocityRadPerSec;
  }

  public TrapezoidProfile.State getCurrentState() {
    // controllers need to run based on real world position due to gravity changing
    return new TrapezoidProfile.State(getRealWorldPositionRadians(), getVelocityRadPerSec());
  }

  public void setGoal(WristGoals wristGoal) {
    if (m_wristGoal == wristGoal) {
      return;
    }
    m_isSetpointAchievedInvalid = true;
    m_wristGoal = wristGoal;
  }

  public WristGoals getGoal() {
    return m_wristGoal;
  }

  public double getSetpointRadians() {
    return m_wristController.getSetpoint();
  }

  @AutoLogOutput(key = "SuperStructure/Wrist/SetpointAchieved")
  public boolean isSetpointAchieved() {
    // always return true if setpoint is lock
    return ((Math.abs(m_goal.position - getRealWorldPositionRadians()) < kWristErrorToleranceRads)
            || m_wristGoal.equals(WristGoals.LOCK)
            || m_wristGoal.equals(WristGoals.STOW))
        && !m_isSetpointAchievedInvalid;
  }

  // Configure SysId
  public void sysIdSetup(SuperStructure superStructure) {
    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(3),
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
