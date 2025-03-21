package frc.robot.subsystems.EndEffector;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.EndEffector.EndEffectorIO.EndEffectorIOInputs;
import java.util.function.DoubleSupplier;

public class EndEffectorIOSpark implements EndEffectorIO {
  private SparkMax m_EndEffectorMotor;
  private RelativeEncoder m_EndEffectorEncoder;
  private SparkMaxConfig m_EndEffectorConfig;
  private Debouncer m_endEffectorConnectedDebounce = new Debouncer(0.5);

  public EndEffectorIOSpark() {
    m_EndEffectorMotor = new SparkMax(EndEffectorConstants.kMotorId, MotorType.kBrushless);
    m_EndEffectorEncoder = m_EndEffectorMotor.getEncoder();

    m_EndEffectorConfig = new SparkMaxConfig();
    m_EndEffectorConfig
        .inverted(EndEffectorConstants.kInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(EndEffectorConstants.kCurrentLimit)
        .voltageCompensation(12.0);
    m_EndEffectorConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    m_EndEffectorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        m_EndEffectorMotor,
        5,
        () ->
            m_EndEffectorMotor.configure(
                m_EndEffectorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void updateInputs(EndEffectorIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        m_EndEffectorMotor,
        m_EndEffectorEncoder::getVelocity,
        (value) -> inputs.velocityRPM = value);
    ifOk(
        m_EndEffectorMotor,
        new DoubleSupplier[] {
          m_EndEffectorMotor::getAppliedOutput, m_EndEffectorMotor::getBusVoltage
        },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(
        m_EndEffectorMotor,
        m_EndEffectorMotor::getOutputCurrent,
        (value) -> inputs.currentAmps = value);
    inputs.endEffectorConnected = m_endEffectorConnectedDebounce.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volt) {
    m_EndEffectorMotor.setVoltage(volt);
  }
}
