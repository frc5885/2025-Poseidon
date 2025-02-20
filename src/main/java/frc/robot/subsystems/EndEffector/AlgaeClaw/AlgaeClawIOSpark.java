package frc.robot.subsystems.EndEffector.AlgaeClaw;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.AlgaeClawConstants;
import java.util.function.DoubleSupplier;

public class AlgaeClawIOSpark implements AlgaeClawIO {
  private SparkMax m_algaeClawMotor;
  private RelativeEncoder m_algaeClawEncoder;
  private SparkMaxConfig m_algaeClawConfig;
  private Debouncer m_algaeClawConnectedDebounce = new Debouncer(0.5);

  public AlgaeClawIOSpark() {
    m_algaeClawMotor = new SparkMax(AlgaeClawConstants.kMotorId, MotorType.kBrushless);
    m_algaeClawEncoder = m_algaeClawMotor.getEncoder();

    m_algaeClawConfig = new SparkMaxConfig();
    m_algaeClawConfig
        .inverted(AlgaeClawConstants.kInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(AlgaeClawConstants.kCurrentLimit)
        .voltageCompensation(12.0);
    m_algaeClawConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    m_algaeClawConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        m_algaeClawMotor,
        5,
        () ->
            m_algaeClawMotor.configure(
                m_algaeClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(AlgaeClawIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        m_algaeClawMotor,
        m_algaeClawEncoder::getPosition,
        (value) -> inputs.positionRotations = value);
    ifOk(m_algaeClawMotor, m_algaeClawEncoder::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        m_algaeClawMotor,
        new DoubleSupplier[] {m_algaeClawMotor::getAppliedOutput, m_algaeClawMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(
        m_algaeClawMotor,
        m_algaeClawMotor::getOutputCurrent,
        (value) -> inputs.currentAmps = value);
    inputs.algaeClawConnected = m_algaeClawConnectedDebounce.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volt) {
    m_algaeClawMotor.setVoltage(volt);
  }
}
