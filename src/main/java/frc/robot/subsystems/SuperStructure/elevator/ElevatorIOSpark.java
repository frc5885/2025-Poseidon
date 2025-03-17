package frc.robot.subsystems.SuperStructure.Elevator;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax m_elevatorSpark1;
  private final SparkMax m_elevatorSpark2;
  private final Debouncer m_elevatorM1ConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer m_elevatorM2ConnectedDebouncer = new Debouncer(0.5);
  private final RelativeEncoder m_elevatorEncoder;

  public ElevatorIOSpark() {
    m_elevatorSpark1 = new SparkMax(kElevatorSparkId1, MotorType.kBrushless);
    m_elevatorSpark2 = new SparkMax(kElevatorSparkId2, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorSpark1.getEncoder();

    SparkMaxConfig elevatorConfig1 = new SparkMaxConfig();
    elevatorConfig1
        .inverted(kElevatorM1Inverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kElevatorMotorCurrentLimit)
        .voltageCompensation(12.0);
    elevatorConfig1
        .encoder
        .positionConversionFactor(kElevatorEncoderPositionFactor)
        .velocityConversionFactor(kElevatorEncoderVelocityFactor)
        .uvwAverageDepth(2);
    elevatorConfig1
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        m_elevatorSpark1,
        5,
        () ->
            m_elevatorSpark1.configure(
                elevatorConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig elevatorConfig2 = elevatorConfig1;
    elevatorConfig2.follow(m_elevatorSpark1, kElevatorM2Opposite);

    tryUntilOk(
        m_elevatorSpark2,
        5,
        () ->
            m_elevatorSpark2.configure(
                elevatorConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // elevator start position
    tryUntilOk(
        m_elevatorSpark1, 5, () -> m_elevatorEncoder.setPosition(kElevatorStartingPositionMeters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    double[] currentAmps = {0.0, 0.0};
    sparkStickyFault = false;
    ifOk(
        m_elevatorSpark1,
        m_elevatorEncoder::getPosition,
        (positionMeters) -> inputs.positionMeters = positionMeters);
    ifOk(
        m_elevatorSpark1,
        m_elevatorEncoder::getVelocity,
        (velocityMetersPerSec) -> inputs.velocityMetersPerSec = velocityMetersPerSec);
    ifOk(
        m_elevatorSpark1,
        new DoubleSupplier[] {m_elevatorSpark1::getAppliedOutput, m_elevatorSpark1::getBusVoltage},
        (appliedVoltage) -> inputs.appliedVolts = appliedVoltage[0] * appliedVoltage[1]);
    ifOk(
        m_elevatorSpark1,
        m_elevatorSpark1::getOutputCurrent,
        (current) -> currentAmps[0] = current);
    inputs.motor1Connected = m_elevatorM1ConnectedDebouncer.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(
        m_elevatorSpark2,
        m_elevatorSpark2::getOutputCurrent,
        (current) -> currentAmps[1] = current);
    inputs.motor2Connected = m_elevatorM2ConnectedDebouncer.calculate(!sparkStickyFault);

    // update the current for both motors
    inputs.currentAmps = currentAmps;
  }

  @Override
  public void setVoltage(double outputVolts) {
    m_elevatorSpark1.setVoltage(outputVolts);
  }

  @Override
  public void setBrakeMode(boolean brakeModeEnabled) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    tryUntilOk(
        m_elevatorSpark1,
        5,
        () ->
            m_elevatorSpark1.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
    tryUntilOk(
        m_elevatorSpark2,
        5,
        () ->
            m_elevatorSpark2.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
