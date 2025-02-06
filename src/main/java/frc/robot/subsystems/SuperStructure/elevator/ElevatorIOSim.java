package frc.robot.subsystems.SuperStructure.elevator;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ElevatorConstants.*;

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
                DCMotor.getNEO(2),
                kElevatorMassKg,
                kElevatorWheelRadiusMeters,
                kElevatorMotorReduction),
            DCMotor.getNEO(2),
            kElevatorLowerBoundMeters,
            kElevatorUpperBoundMeters,
            true,
            kElevatorStartingPositionMeters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    m_elevatorSim.update(0.02);
    inputs.elevatorM1Connected = true;
    inputs.elevatorM2Connected = true;
    inputs.elevatorPositionMeters = m_elevatorSim.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = m_elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorAppliedVolts = m_appliedVolts;
    inputs.elevatorCurrentAmps =
        new double[] {m_elevatorSim.getCurrentDrawAmps(), m_elevatorSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double outputVolts) {
    m_appliedVolts = MathUtil.clamp(outputVolts, -12.0, 12.0);
    m_elevatorSim.setInputVoltage(m_appliedVolts);
  }
}
