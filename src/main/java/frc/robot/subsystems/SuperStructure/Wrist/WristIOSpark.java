package frc.robot.subsystems.SuperStructure.Wrist;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.kArmStartingPositionRadians;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class WristIOSpark implements WristIO {
  private final SparkMax m_wristSpark;
  private final Debouncer m_wristConnectedDebouncer = new Debouncer(0.5);
  private final RelativeEncoder m_wristEncoder;

  public WristIOSpark() {
    m_wristSpark = new SparkMax(kWristSparkId, MotorType.kBrushless);
    m_wristEncoder = m_wristSpark.getEncoder();

    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig
        .inverted(kWristInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kWristMotorCurrentLimit)
        .voltageCompensation(12.0);
    wristConfig
        .encoder
        .positionConversionFactor(kWristEncoderPositionFactor)
        .velocityConversionFactor(kWristEncoderVelocityFactor)
        .uvwAverageDepth(2);
    wristConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        m_wristSpark,
        5,
        () ->
            m_wristSpark.configure(
                wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(
        m_wristSpark,
        5,
        () ->
            m_wristEncoder.setPosition(
                kWristStartingPositionRadians - kArmStartingPositionRadians));
  }

  @Override
  public void updateInputs(WristIOInputs inputs, DoubleSupplier armAngleSupplier) {
    double armAngleRads = armAngleSupplier.getAsDouble();

    sparkStickyFault = false;
    ifOk(
        m_wristSpark,
        m_wristEncoder::getPosition,
        (positionRads) -> inputs.positionRads = positionRads);
    ifOk(
        m_wristSpark,
        m_wristEncoder::getPosition,
        (positionRads) -> inputs.realWorldPositionRads = positionRads + armAngleRads);
    ifOk(
        m_wristSpark,
        m_wristEncoder::getVelocity,
        (velocityRadPerSec) -> inputs.wristVelocityRadPerSec = velocityRadPerSec);
    ifOk(
        m_wristSpark,
        new DoubleSupplier[] {m_wristSpark::getAppliedOutput, m_wristSpark::getBusVoltage},
        (appliedVoltage) -> inputs.wristAppliedVolts = appliedVoltage[0] * appliedVoltage[1]);
    ifOk(
        m_wristSpark,
        m_wristSpark::getOutputCurrent,
        (current) -> inputs.wristCurrentAmps = current);
    inputs.wristConnected = m_wristConnectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setVoltage(double outputVolts) {
    m_wristSpark.setVoltage(outputVolts);
  }
}
