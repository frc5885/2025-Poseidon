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
import frc.robot.subsystems.Collector.CollectorConstants.IntakeConstants;
import java.util.function.DoubleSupplier;

public class IntakeIOSpark implements IntakeIO {
  private SparkMax intakeMotor1;
  private SparkMax intakeMotor2;
  private RelativeEncoder intake1Encoder;

  private final Debouncer intakeConnectedDebounce1 = new Debouncer(0.5);
  private final Debouncer intakeConnectedDebounce2 = new Debouncer(0.5);

  public IntakeIOSpark() {
    intakeMotor1 = new SparkMax(IntakeConstants.intakeId1, MotorType.kBrushless);
    intakeMotor2 = new SparkMax(IntakeConstants.intakeId2, MotorType.kBrushless);

    intake1Encoder = intakeMotor1.getEncoder();

    SparkMaxConfig intake1Config = new SparkMaxConfig();
    intake1Config
        .inverted(IntakeConstants.intakeMotor1Inverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.intakeMotorCurrentLimit)
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
        intakeMotor1,
        5,
        () ->
            intakeMotor1.configure(
                intake1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig intake2Config = intake1Config;
    intake2Config.follow(intakeMotor1, IntakeConstants.intakeMotor2Inverted);

    tryUntilOk(
        intakeMotor2,
        5,
        () ->
            intakeMotor2.configure(
                intake2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(IntakeIOInputs inputs) {
    double[] current = {0.0, 0.0};
    sparkStickyFault = false;
    ifOk(intakeMotor1, intake1Encoder::getPosition, (value) -> inputs.positionRotations = value);
    ifOk(intakeMotor1, intake1Encoder::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        intakeMotor1,
        new DoubleSupplier[] {intakeMotor1::getAppliedOutput, intakeMotor1::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(intakeMotor1, intakeMotor1::getOutputCurrent, (value) -> current[0] = value);
    inputs.intake1Connected = intakeConnectedDebounce1.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(intakeMotor2, intakeMotor2::getOutputCurrent, (value) -> current[1] = value);
    inputs.intake2Connected = intakeConnectedDebounce2.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts) {
    intakeMotor1.setVoltage(volts);
  }
}
