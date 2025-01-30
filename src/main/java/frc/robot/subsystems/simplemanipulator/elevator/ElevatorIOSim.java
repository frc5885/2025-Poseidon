package frc.robot.subsystems.simplemanipulator.elevator;

import static frc.robot.subsystems.simplemanipulator.ManipulatorConstants.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.TunablePIDController;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim m_elevatorSim;
  // TODO maybe add a mechanism2d

  private TrapezoidProfile m_elevatorSimProfile;
  private TrapezoidProfile.State m_goalSim;
  private TunablePIDController m_elevatorSimController;
  private ElevatorFeedforward m_elevatorSimFeedforward;

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
            kElevatorLowerBoundMeters);
    m_elevatorSimProfile =
        new TrapezoidProfile(new Constraints(kElevatorMaxVelocity, kElevatorMaxAcceleration));
    m_goalSim = getCurrentState();
    m_elevatorSimController =
        new TunablePIDController(
            elevatorSimKp,
            0.0,
            elevatorSimKd,
            kElevatorErrorToleranceMeters,
            "ElevatorSimPID",
            true);
    m_elevatorSimFeedforward = new ElevatorFeedforward(0.0, elevatorSimKg, elevatorSimKv);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (m_isClosedLoop) {
      TrapezoidProfile.State current = getCurrentState();
      TrapezoidProfile.State setpoint = m_elevatorSimProfile.calculate(0.02, current, m_goalSim);
      m_elevatorAppliedVolts =
          m_elevatorSimFeedforward.calculate(setpoint.velocity)
              + m_elevatorSimController.calculate(current.position, setpoint.position);
    }
    m_elevatorSim.setInputVoltage(MathUtil.clamp(m_elevatorAppliedVolts, -12.0, 12.0));
    m_elevatorSim.update(0.02);

    inputs.elevatorConnected = true;
    inputs.elevatorPositionMeters = m_elevatorSim.getPositionMeters();
    inputs.elevatorVelocityMetersPerSec = m_elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorErrorMeters = m_goalSim.position - inputs.elevatorPositionMeters;
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
    if (m_goalSim.position != positionMeters) {
      m_goalSim = new TrapezoidProfile.State(positionMeters, 0.0);
    }
  }

  private TrapezoidProfile.State getCurrentState() {
    return new State(m_elevatorSim.getPositionMeters(), m_elevatorSim.getVelocityMetersPerSecond());
  }

  @Override
  public void stop() {
    m_isClosedLoop = false;
    m_elevatorAppliedVolts = 0.0;
  }
}
