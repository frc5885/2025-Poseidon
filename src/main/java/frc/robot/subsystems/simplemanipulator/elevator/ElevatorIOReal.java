package frc.robot.subsystems.simplemanipulator.elevator;

import static frc.robot.subsystems.simplemanipulator.ManipulatorConstants.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;

public class ElevatorIOReal implements ElevatorIO {
  private final SparkBase m_elevatorSpark;
  private final RelativeEncoder m_elevatorEncoder;
  private final SparkClosedLoopController m_elevatorController;

  public ElevatorIOReal(int elevatorSparkId) {
    m_elevatorSpark = new SparkMax(elevatorSparkId, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorSpark.getEncoder();
    m_elevatorController = m_elevatorSpark.getClosedLoopController();
    // TODO add more configs
    var elevatorConfig = new SparkMaxConfig();
    elevatorConfig
        .encoder
        .positionConversionFactor(kElevatorEncoderPositionFactor)
        .velocityConversionFactor(kElevatorEncoderVelocityFactor);
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
  }

  @Override
  public void setElevatorOpenLoop(double outputVolts) {
    m_elevatorSpark.setVoltage(outputVolts);
  }

  @Override
  public void setElevatorPosition(double positionMeters) {
    m_elevatorController.setReference(positionMeters, ControlType.kPosition);
  }

  @Override
  public void stop() {
    m_elevatorSpark.setVoltage(0.0);
  }
}
