package frc.robot.subsystems.simplemanipulator.elevator;

import static frc.robot.subsystems.simplemanipulator.ManipulatorConstants.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim m_elevatorSim;

  private double m_appliedVolts;

  public ElevatorIOSim() {
    m_elevatorSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getNEO(1),
                kElevatorMassKg,
                kElevatorWheelRadiusMeters,
                kElevatorMotorReduction),
            DCMotor.getNEO(1),
            kElevatorLowerBoundMeters,
            kElevatorUpperBoundMeters,
            true,
            kElevatorLowerBoundMeters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    m_elevatorSim.update(0.02);
    inputs.elevatorConnected = true;
    inputs.elevatorPositionMeters = m_elevatorSim.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = m_elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorAppliedVolts = m_appliedVolts;
    inputs.elevatorCurrentAmps = m_elevatorSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double outputVolts) {
    m_appliedVolts = MathUtil.clamp(outputVolts, -12.0, 12.0);
    m_elevatorSim.setInputVoltage(m_appliedVolts);
  }
}
