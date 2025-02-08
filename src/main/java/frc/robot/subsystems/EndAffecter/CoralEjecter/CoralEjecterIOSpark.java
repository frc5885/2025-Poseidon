package frc.robot.subsystems.EndAffecter.CoralEjecter;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.EndAffecter.EndAffecterConstant.CoralEjecterConstants;
import java.util.function.DoubleSupplier;

public class CoralEjecterIOSpark implements CoralEjecterIO {

  private SparkMax m_coralEjecterMoter;
  private RelativeEncoder m_coralEjectEncoder;
  private SparkMaxConfig m_coralEjectConfig;
  private final Debouncer intakeConnectedDebounce1 = new Debouncer(0.5);

  public CoralEjecterIOSpark() {
    m_coralEjecterMoter = new SparkMax(CoralEjecterConstants.CoralEjecterId, MotorType.kBrushless);

    m_coralEjectEncoder = m_coralEjecterMoter.getEncoder();

    m_coralEjectConfig = new SparkMaxConfig();

    m_coralEjectConfig
        .inverted(CoralEjecterConstants.CoralEjecterInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(CoralEjecterConstants.CoralEjecterCurrentLimit)
        .voltageCompensation(12.0);
    m_coralEjectConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    m_coralEjectConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        m_coralEjecterMoter,
        5,
        () ->
            m_coralEjecterMoter.configure(
                m_coralEjectConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    tryUntilOk(
        m_coralEjecterMoter,
        5,
        () ->
            m_coralEjecterMoter.configure(
                m_coralEjectConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void updateInputs(CoralEjecterInputs inputs) {
    double[] current = {0.0, 0.0};
    sparkStickyFault = false;
    ifOk(
        m_coralEjecterMoter,
        m_coralEjectEncoder::getPosition,
        (value) -> inputs.positionRotations = value);
    ifOk(
        m_coralEjecterMoter,
        m_coralEjectEncoder::getVelocity,
        (value) -> inputs.velocityRPM = value);
    ifOk(
        m_coralEjecterMoter,
        new DoubleSupplier[] {
          m_coralEjecterMoter::getAppliedOutput, m_coralEjecterMoter::getBusVoltage
        },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(m_coralEjecterMoter, m_coralEjecterMoter::getOutputCurrent, (value) -> current[0] = value);
    inputs.CoralEjecterConnected = intakeConnectedDebounce1.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts) {
    m_coralEjecterMoter.setVoltage(volts);
  }
}
