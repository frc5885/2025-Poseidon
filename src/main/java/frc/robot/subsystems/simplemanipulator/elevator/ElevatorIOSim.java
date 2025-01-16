package frc.robot.subsystems.simplemanipulator.elevator;

import static frc.robot.subsystems.simplemanipulator.ManipulatorConstants.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim m_elevatorSim;
  private final PIDController m_controller;

  private boolean m_isClosedLoop = false;
  private double m_elevatorAppliedVolts = 0.0;

  public ElevatorIOSim() {
    m_elevatorSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getNEO(1), 4.0, kElevatorWheelRadiusMeters, kElevatorMotorReduction),
            DCMotor.getNEO(1),
            kElevatorLowerBoundMeters,
            kElevatorUpperBoundMeters,
            true,
            0.0);
    m_controller = new PIDController(elevatorSimKp, 0.0, elevatorSimKd);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (m_isClosedLoop) {
      m_elevatorAppliedVolts = m_controller.calculate(m_elevatorSim.getPositionMeters());
    }

    m_elevatorAppliedVolts = MathUtil.clamp(m_elevatorAppliedVolts + elevatorSimKg, -12.0, 12.0);
    m_elevatorSim.setInputVoltage(m_elevatorAppliedVolts);
    m_elevatorSim.update(0.02);

    inputs.elevatorConnected = true;
    inputs.elevatorPositionMeters = m_elevatorSim.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = m_elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorAppliedVolts = m_elevatorAppliedVolts;
    inputs.elevatorCurrentAmps = m_elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public void setElevatorOpenLoop(double outputVolts) {
    m_isClosedLoop = false;
    m_elevatorAppliedVolts = outputVolts;
  }

  @Override
  public void setElevatorPosition(double positionMeters) {
    m_isClosedLoop = true;
    m_controller.setSetpoint(positionMeters);
  }

  @Override
  public void stop() {
    m_isClosedLoop = false;
    m_elevatorAppliedVolts = 0.0;
  }
}
