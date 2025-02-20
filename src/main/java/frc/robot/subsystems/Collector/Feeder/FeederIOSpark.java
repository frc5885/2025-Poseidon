package frc.robot.subsystems.Collector.Feeder;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.Collector.CollectorConstants.FeederConstants;
import java.util.function.DoubleSupplier;

public class FeederIOSpark implements FeederIO {
  private SparkMax m_feederMotor;
  private RelativeEncoder m_feederEncoder;
  private final Debouncer m_feederConnectedDebounce = new Debouncer(0.5);

  public FeederIOSpark() {
    m_feederMotor = new SparkMax(FeederConstants.kMotorId, MotorType.kBrushless);
    m_feederEncoder = m_feederMotor.getEncoder();

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig
        .inverted(FeederConstants.kInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(FeederConstants.kCurrentLimit)
        .voltageCompensation(12.0);
    feederConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    feederConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        m_feederMotor,
        5,
        () ->
            m_feederMotor.configure(
                feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(FeederIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(m_feederMotor, m_feederEncoder::getPosition, (value) -> inputs.positionRotations = value);
    ifOk(m_feederMotor, m_feederEncoder::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        m_feederMotor,
        new DoubleSupplier[] {m_feederMotor::getAppliedOutput, m_feederMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(m_feederMotor, m_feederMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.feederConnected = m_feederConnectedDebounce.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts) {
    m_feederMotor.setVoltage(volts);
  }
}
