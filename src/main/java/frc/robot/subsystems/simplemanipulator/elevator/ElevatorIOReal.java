package frc.robot.subsystems.simplemanipulator.elevator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import java.util.function.DoubleSupplier;

public class ElevatorIOReal implements ElevatorIO {
  private final SparkBase m_elevatorSpark;
  private final RelativeEncoder m_elevatorEncoder;
  private final SparkClosedLoopController m_elevatorController;

  public ElevatorIOReal(int elevatorSparkId) {
    m_elevatorSpark = new SparkMax(elevatorSparkId, MotorType.kBrushless);
    m_elevatorEncoder = m_elevatorSpark.getEncoder();
    m_elevatorController = m_elevatorSpark.getClosedLoopController();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    ifOk(
        m_elevatorSpark,
        m_elevatorEncoder::getPosition,
        (position) -> inputs.elevatorPosition = position);
    ifOk(
        m_elevatorSpark,
        m_elevatorEncoder::getVelocity,
        (velocity) -> inputs.elevatorVelocity = velocity);
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
  public void setElevatorPosition(double position) {
    double setpoint = position;
    m_elevatorController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void stop() {
    m_elevatorSpark.setVoltage(0.0);
  }
}
