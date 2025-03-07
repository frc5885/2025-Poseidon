package frc.robot.subsystems.Collector.Intake;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.subsystems.Collector.CollectorConstants.IntakeConstants;
import java.util.function.DoubleSupplier;

public class IntakeIOSpark implements IntakeIO {
  private SparkMax m_intakeMotor1;
  private SparkMax m_intakeMotor2;
  private RelativeEncoder m_intakeEncoder1;
  private final Debouncer m_intakeConnectedDebounce1 = new Debouncer(0.5);
  private final Debouncer m_intakeConnectedDebounce2 = new Debouncer(0.5);

  private final Solenoid m_leftSolenoid =
      new Solenoid(
          IntakeConstants.kPneumaticHubCanID,
          PneumaticsModuleType.REVPH,
          IntakeConstants.kSolenoidId1);
  private final Solenoid m_rightSolenoid =
      new Solenoid(
          IntakeConstants.kPneumaticHubCanID,
          PneumaticsModuleType.REVPH,
          IntakeConstants.kSolenoidId2);

  public IntakeIOSpark() {
    m_intakeMotor1 = new SparkMax(IntakeConstants.kMotorId1, MotorType.kBrushless);
    m_intakeMotor2 = new SparkMax(IntakeConstants.kMotorId2, MotorType.kBrushless);

    m_intakeEncoder1 = m_intakeMotor1.getEncoder();

    SparkMaxConfig intake1Config = new SparkMaxConfig();
    intake1Config
        .inverted(IntakeConstants.kMotor1Inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.kCurrentLimit)
        .voltageCompensation(12.0);
    intake1Config.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    intake1Config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        m_intakeMotor1,
        5,
        () ->
            m_intakeMotor1.configure(
                intake1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig intake2Config = intake1Config;
    intake2Config.follow(m_intakeMotor1, IntakeConstants.kMotor2Opposite);

    tryUntilOk(
        m_intakeMotor2,
        5,
        () ->
            m_intakeMotor2.configure(
                intake2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // retract intake on startup
    retractIntake();
  }

  public void updateInputs(IntakeIOInputs inputs) {
    double[] current = {0.0, 0.0};
    sparkStickyFault = false;
    ifOk(
        m_intakeMotor1, m_intakeEncoder1::getPosition, (value) -> inputs.positionRotations = value);
    ifOk(m_intakeMotor1, m_intakeEncoder1::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        m_intakeMotor1,
        new DoubleSupplier[] {m_intakeMotor1::getAppliedOutput, m_intakeMotor1::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(m_intakeMotor1, m_intakeMotor1::getOutputCurrent, (value) -> current[0] = value);
    inputs.motor1Connected = m_intakeConnectedDebounce1.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(m_intakeMotor2, m_intakeMotor2::getOutputCurrent, (value) -> current[1] = value);
    inputs.motor2Connected = m_intakeConnectedDebounce2.calculate(!sparkStickyFault);
    inputs.currentAmps = current;
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts) {
    m_intakeMotor1.setVoltage(volts);
  }

  public void extendIntake() {
    m_leftSolenoid.set(true);
    m_rightSolenoid.set(true);
  }

  public void retractIntake() {
    m_leftSolenoid.set(false);
    m_rightSolenoid.set(false);
  }
}
