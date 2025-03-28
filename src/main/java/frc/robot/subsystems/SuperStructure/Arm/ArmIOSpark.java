package frc.robot.subsystems.SuperStructure.Arm;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.*;
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

public class ArmIOSpark implements ArmIO {
  private final SparkMax m_armSpark;
  private final Debouncer m_armConnectedDebouncer = new Debouncer(0.5);
  private final RelativeEncoder m_armEncoder;

  public ArmIOSpark() {
    m_armSpark = new SparkMax(kArmSparkId, MotorType.kBrushless);
    m_armEncoder = m_armSpark.getEncoder();

    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig
        .inverted(kArmInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kArmMotorCurrentLimit)
        .voltageCompensation(12.0);
    armConfig
        .encoder
        .positionConversionFactor(kArmEncoderPositionFactor)
        .velocityConversionFactor(kArmEncoderVelocityFactor)
        .uvwAverageDepth(2);
    armConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        m_armSpark,
        5,
        () ->
            m_armSpark.configure(
                armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // arm start position
    tryUntilOk(m_armSpark, 5, () -> m_armEncoder.setPosition(kArmStartingPositionRads));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        m_armSpark,
        m_armEncoder::getPosition,
        (positionRads) -> inputs.positionRads = positionRads);
    ifOk(
        m_armSpark,
        m_armEncoder::getVelocity,
        (velocityRadsPerSec) -> inputs.velocityRadsPerSec = velocityRadsPerSec);
    ifOk(
        m_armSpark,
        new DoubleSupplier[] {m_armSpark::getAppliedOutput, m_armSpark::getBusVoltage},
        (appliedVoltage) -> inputs.appliedVolts = appliedVoltage[0] * appliedVoltage[1]);
    ifOk(m_armSpark, m_armSpark::getOutputCurrent, (current) -> inputs.currentAmps = current);
    inputs.motorConnected = m_armConnectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setVoltage(double outputVolts) {
    m_armSpark.setVoltage(outputVolts);
  }

  @Override
  public void setBrakeMode(boolean brakeModeEnabled) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    tryUntilOk(
        m_armSpark,
        5,
        () ->
            m_armSpark.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }
}
