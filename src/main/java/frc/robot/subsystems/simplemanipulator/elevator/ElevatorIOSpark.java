package frc.robot.subsystems.simplemanipulator.elevator;

import static frc.robot.subsystems.simplemanipulator.ManipulatorConstants.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.TunablePIDController;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkBase m_elevatorSpark;
  private final RelativeEncoder m_elevatorEncoder;
  private final SparkClosedLoopController m_elevatorController;
  private TrapezoidProfile m_elevatorProfile;
  private TrapezoidProfile.State m_goal;
  private TunablePIDController m_elevatorPIDController;
  private ElevatorFeedforward m_elevatorFeedforward;

  public ElevatorIOSpark(int elevatorSparkId) {
    m_elevatorSpark = new SparkMax(elevatorSparkId, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorSpark.getEncoder();
    m_elevatorController = m_elevatorSpark.getClosedLoopController();

    m_elevatorProfile =
        new TrapezoidProfile(new Constraints(kElevatorMaxVelocity, kElevatorMaxAcceleration));
    m_goal = getCurrentState();
    // TODO remember to set the turnMode on
    m_elevatorPIDController =
        new TunablePIDController(
            elevatorKp, 0.0, elevatorKd, kElevatorErrorToleranceMeters, "ElevatorPID", false);
    m_elevatorFeedforward = new ElevatorFeedforward(elevatorKs, elevatorKg, elevatorKv);

    // TODO keep some of this?
    var elevatorConfig = new SparkMaxConfig();
    elevatorConfig
        .inverted(kElevatorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kElevatorMotorCurrentLimit)
        .voltageCompensation(12.0);
    elevatorConfig
        .encoder
        .positionConversionFactor(kElevatorEncoderPositionFactor)
        .velocityConversionFactor(kElevatorEncoderVelocityFactor)
        .uvwAverageDepth(2);
    elevatorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // PositionWrapping?
        .pidf(elevatorKp, 0.0, elevatorKd, elevatorKv);
    // MaxMotion?
    elevatorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        m_elevatorSpark,
        5,
        () ->
            m_elevatorSpark.configure(
                elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(m_elevatorSpark, 5, () -> m_elevatorEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    ifOk(
        m_elevatorSpark,
        m_elevatorEncoder::getPosition,
        (positionMeters) -> inputs.elevatorPositionMeters = positionMeters);
    ifOk(
        m_elevatorSpark,
        m_elevatorEncoder::getVelocity,
        (velocityMetersPerSec) -> inputs.elevatorVelocityMetersPerSec = velocityMetersPerSec);
    ifOk(
        m_elevatorSpark,
        new DoubleSupplier[] {m_elevatorSpark::getAppliedOutput, m_elevatorSpark::getBusVoltage},
        (appliedVoltage) -> inputs.elevatorAppliedVolts = appliedVoltage[0] * appliedVoltage[1]);
    ifOk(
        m_elevatorSpark,
        m_elevatorSpark::getOutputCurrent,
        (current) -> inputs.elevatorCurrentAmps = current);
  }

  @Override
  public void setElevatorOpenLoop(double outputVolts) {
    m_elevatorSpark.setVoltage(outputVolts);
  }

  @Override
  public void setElevatorPosition(double positionMeters) {
    // m_elevatorController.setReference(positionMeters, ControlType.kPosition);

    if (m_goal.position != positionMeters) {
      m_goal = new TrapezoidProfile.State(positionMeters, 0.0);
    }
    TrapezoidProfile.State current = getCurrentState();
    TrapezoidProfile.State setpoint = m_elevatorProfile.calculate(0.02, current, m_goal);
    m_elevatorSpark.setVoltage(
        m_elevatorPIDController.calculate(current.position, setpoint.position)
            + m_elevatorFeedforward.calculate(setpoint.velocity));
  }

  @Override
  public void stop() {
    m_elevatorSpark.setVoltage(0.0);
  }

  private TrapezoidProfile.State getCurrentState() {
    return new State(m_elevatorEncoder.getPosition(), m_elevatorEncoder.getVelocity());
  }
}
