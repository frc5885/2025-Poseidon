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
  private RelativeEncoder m_coralEjectEncoder;
  private SparkMaxConfig m_coralEjectConfig;
  private final Debouncer coralEjectorConnectedDebounce = new Debouncer(0.5);

  public CoralEjectorIOSpark() {
    m_coralEjectorMotor = new SparkMax(CoralEjectorConstants.CoralEjectorId, MotorType.kBrushless);

    m_coralEjectEncoder = m_coralEjectorMotor.getEncoder();

    m_coralEjectConfig = new SparkMaxConfig();

    m_coralEjectConfig
        .inverted(CoralEjectorConstants.CoralEjectorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(CoralEjectorConstants.CoralEjectorCurrentLimit)
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
        m_coralEjectorMotor,
        5,
        () ->
            m_coralEjectorMotor.configure(
                m_coralEjectConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
  }

  public void updateInputs(CoralEjectorInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        m_coralEjectorMotor,
        m_coralEjectEncoder::getPosition,
        (value) -> inputs.positionRotations = value);
    ifOk(
        m_coralEjectorMotor,
        m_coralEjectEncoder::getVelocity,
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
    inputs.coralEjectorConnected = coralEjectorConnectedDebounce.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts) {
    m_coralEjectorMotor.setVoltage(volts);
  }
}
