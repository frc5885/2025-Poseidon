package frc.robot.subsystems.simplemanipulator.elevator;

import static frc.robot.subsystems.simplemanipulator.ManipulatorConstants.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.DoubleSupplier;

public class ElevatorIOSpark implements ElevatorIO {
  private final SparkMax m_elevatorSpark;
  private final Debouncer m_elevatorConnectedDebouncer = new Debouncer(0.5);
  private final RelativeEncoder m_elevatorEncoder;

  public ElevatorIOSpark() {
    m_elevatorSpark = new SparkMax(kElevatorSparkId, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorSpark.getEncoder();

    var elevatorConfig = new SparkMaxConfig();
    elevatorConfig
        .inverted(kElevatorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kElevatorMotorCurrentLimit)
        .voltageCompensation(12.0);
    elevatorConfig
        .encoder
        .positionConversionFactor(kElevatorEncoderPositionFactor)
        .velocityConversionFactor(kElevatorEncoderVelocityFactor)
        .uvwAverageDepth(2);
    elevatorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    tryUntilOk(
        m_elevatorSpark,
        5,
        () ->
            m_elevatorSpark.configure(
                elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(m_elevatorSpark, 5, () -> m_elevatorEncoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        m_elevatorSpark,
        m_elevatorEncoder::getPosition,
        (positionMeters) -> inputs.elevatorPositionMeters = positionMeters);
    ifOk(
        m_elevatorSpark,
        m_elevatorEncoder::getVelocity,
        (velocityMetersPerSec) -> inputs.elevatorVelocityMetersPerSec = velocityMetersPerSec);
    ifOk(
        m_elevatorSpark,
        new DoubleSupplier[] {m_elevatorSpark::getAppliedOutput, m_elevatorSpark::getBusVoltage},
        (appliedVoltage) -> inputs.elevatorAppliedVolts = appliedVoltage[0] * appliedVoltage[1]);
    ifOk(
        m_elevatorSpark,
        m_elevatorSpark::getOutputCurrent,
        (current) -> inputs.elevatorCurrentAmps = current);
    inputs.elevatorConnected = m_elevatorConnectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setVoltage(double outputVolts) {
    m_elevatorSpark.setVoltage(outputVolts);
  }
}
