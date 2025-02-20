package frc.robot.subsystems.EndEffector.CoralEjector;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.CoralEjectorConstants;
import java.util.function.DoubleSupplier;

public class CoralEjectorIOSpark implements CoralEjectorIO {

  private SparkMax m_coralEjectorMotor;
  private RelativeEncoder m_coralEjectorEncoder;
  private SparkMaxConfig m_coralEjectorConfig;
  private final Debouncer m_coralEjectorConnectedDebounce = new Debouncer(0.5);

  public CoralEjectorIOSpark() {
    m_coralEjectorMotor = new SparkMax(CoralEjectorConstants.kMotorId, MotorType.kBrushless);

    m_coralEjectorEncoder = m_coralEjectorMotor.getEncoder();

    m_coralEjectorConfig = new SparkMaxConfig();

    m_coralEjectorConfig
        .inverted(CoralEjectorConstants.kInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(CoralEjectorConstants.kCurrentLimit)
        .voltageCompensation(12.0);
    m_coralEjectorConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    m_coralEjectorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        m_coralEjectorMotor,
        5,
        () ->
            m_coralEjectorMotor.configure(
                m_coralEjectorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void updateInputs(CoralEjectorIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        m_coralEjectorMotor,
        m_coralEjectorEncoder::getPosition,
        (value) -> inputs.positionRotations = value);
    ifOk(
        m_coralEjectorMotor,
        m_coralEjectorEncoder::getVelocity,
        (value) -> inputs.velocityRPM = value);
    ifOk(
        m_coralEjectorMotor,
        new DoubleSupplier[] {
          m_coralEjectorMotor::getAppliedOutput, m_coralEjectorMotor::getBusVoltage
        },
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(
        m_coralEjectorMotor,
        m_coralEjectorMotor::getOutputCurrent,
        (value) -> inputs.currentAmps = value);
    inputs.coralEjectorConnected = m_coralEjectorConnectedDebounce.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts) {
    m_coralEjectorMotor.setVoltage(volts);
  }
}
