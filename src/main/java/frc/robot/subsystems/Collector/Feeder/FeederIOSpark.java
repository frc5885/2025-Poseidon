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
  private SparkMax feederMotor;
  private RelativeEncoder feederEncoder;

  private final Debouncer feederConnectedDebounce = new Debouncer(0.5);

  public FeederIOSpark() {
    feederMotor = new SparkMax(FeederConstants.feederId, MotorType.kBrushless);
    feederEncoder = feederMotor.getEncoder();

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig
        .inverted(FeederConstants.feederMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(FeederConstants.feederMotorCurrentLimit)
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
        feederMotor,
        5,
        () ->
            feederMotor.configure(
                feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  public void updateInputs(FeederIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(feederMotor, feederEncoder::getPosition, (value) -> inputs.positionRotations = value);
    ifOk(feederMotor, feederEncoder::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        feederMotor,
        new DoubleSupplier[] {feederMotor::getAppliedOutput, feederMotor::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(feederMotor, feederMotor::getOutputCurrent, (value) -> inputs.currentAmps = value);
    inputs.feederConnected = feederConnectedDebounce.calculate(!sparkStickyFault);
  }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts) {
    feederMotor.setVoltage(volts);
  }
}
