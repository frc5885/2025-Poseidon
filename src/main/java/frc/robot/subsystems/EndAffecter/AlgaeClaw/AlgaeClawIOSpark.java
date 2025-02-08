package frc.robot.subsystems.EndAffecter.AlgaeClaw;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.Collector.CollectorConstants.AlgaeClawConstants;
import frc.robot.subsystems.EndAffecter.CoralEjecter.CoralEjecterIO.CoralEjecterInputs;
import java.util.function.DoubleSupplier;

public class AlgaeClawIOSpark implements AlgaeClawIO {
  private SparkMax m_AlgaeClawMotor;
  private RelativeEncoder m_AlgeaClawEncoder;
  private SparkMaxConfig m_AlgaeClawConfig;
  private Debouncer AlgeaClawConnectedDebounce = new Debouncer(0.5);

  public AlgaeClawIOSpark() {

    m_AlgaeClawMotor = new SparkMax(AlgaeClawConstants.AlgaeClawId, MotorType.kBrushless);

    m_AlgeaClawEncoder = m_AlgaeClawMotor.getEncoder();

    m_AlgaeClawConfig = new SparkMaxConfig();
    m_AlgaeClawConfig
        .inverted(AlgaeClawConstants.AlgaeClawInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(AlgaeClawConstants.AlgaeClawCurrentLimit)
        .voltageCompensation(12.0);
    m_AlgaeClawConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    m_AlgaeClawConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        m_AlgaeClawMotor,
        5,
        () ->
            m_AlgaeClawMotor.configure(
                m_AlgaeClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(
        m_AlgaeClawMotor,
        5,
        () ->
            m_AlgaeClawMotor.configure(
                m_AlgaeClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(CoralEjecterInputs inputs) {
    double[] current = {0.0, 0.0};
    sparkStickyFault = false;
    ifOk(
        m_AlgaeClawMotor,
        m_AlgeaClawEncoder::getPosition,
        (value) -> inputs.positionRotations = value);
    ifOk(m_AlgaeClawMotor, m_AlgeaClawEncoder::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        m_AlgaeClawMotor,
        new DoubleSupplier[] {m_AlgaeClawMotor::getAppliedOutput, m_AlgaeClawMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(m_AlgaeClawMotor, m_AlgaeClawMotor::getOutputCurrent, (value) -> current[0] = value);
    inputs.CoralEjecterConnected = AlgeaClawConnectedDebounce.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volt) {
    m_AlgaeClawMotor.setVoltage(volt);
  }
}
