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
  private SparkMax m_feederMotor1;
  private SparkMax m_feederMotor2;
  private RelativeEncoder m_feederEncoder1;
  private final Debouncer m_feederConnectedDebounce1 = new Debouncer(0.5);
  private final Debouncer m_feederConnectedDebounce2 = new Debouncer(0.5);

  public FeederIOSpark() {
    m_feederMotor1 = new SparkMax(FeederConstants.kMotorId1, MotorType.kBrushless);
    m_feederMotor2 = new SparkMax(FeederConstants.kMotorId2, MotorType.kBrushless);
    m_feederEncoder1 = m_feederMotor1.getEncoder();

    SparkMaxConfig feeder1Config = new SparkMaxConfig();
    feeder1Config
        .inverted(FeederConstants.kMotor1Inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(FeederConstants.kCurrentLimit)
        .voltageCompensation(12.0);
    feeder1Config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    feeder1Config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        m_feederMotor1,
        5,
        () ->
            m_feederMotor1.configure(
                feeder1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig feeder2Config = feeder1Config;
    feeder2Config.follow(m_feederMotor1, FeederConstants.kMotor2Inverted != FeederConstants.kMotor2Inverted);

    tryUntilOk(
        m_feederMotor2,
        5,
        () ->
            m_feederMotor2.configure(
                feeder2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(FeederIOInputs inputs) {
    double[] current = {0.0, 0.0};
    sparkStickyFault = false;
    ifOk(
        m_feederMotor1, m_feederEncoder1::getPosition, (value) -> inputs.positionRotations = value);
    ifOk(m_feederMotor1, m_feederEncoder1::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        m_feederMotor1,
        new DoubleSupplier[] {m_feederMotor1::getAppliedOutput, m_feederMotor1::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(m_feederMotor1, m_feederMotor1::getOutputCurrent, (value) -> current[0] = value);
    inputs.motor1Connected = m_feederConnectedDebounce1.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(m_feederMotor2, m_feederMotor2::getOutputCurrent, (value) -> current[1] = value);
    inputs.motor2Connected = m_feederConnectedDebounce2.calculate(!sparkStickyFault);
    inputs.currentAmps = current;
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts) {
    m_feederMotor1.setVoltage(volts);
  }
}
