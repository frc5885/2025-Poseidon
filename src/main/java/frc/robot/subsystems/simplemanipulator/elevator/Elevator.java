package frc.robot.subsystems.simplemanipulator.elevator;

import static frc.robot.subsystems.simplemanipulator.ManipulatorConstants.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO m_io;
  private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();
  private ElevatorLevel m_elevatorLevel = ElevatorLevel.L1;

  public Elevator(ElevatorIO io) {
    m_io = io;
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("SimpleManipulator/Elevator", m_inputs);

    runElevatorSetpoint(
        m_elevatorLevel != null ? m_elevatorLevel.setpointMeters : getPositionMeters());
  }

  public void runElevatorOpenLoop(double outputVolts) {
    // TODO MUST match the real implementation!
    if (outputVolts > 0) {
      m_io.setElevatorOpenLoop(isWithinUpperBound(getPositionMeters()) ? outputVolts : 0.0);
    } else if (outputVolts < 0) {
      m_io.setElevatorOpenLoop(isWithinLowerBound(getPositionMeters()) ? outputVolts : 0.0);
    } else {
      m_io.setElevatorOpenLoop(outputVolts);
    }
  }

  public void runElevatorSetpoint(double setpointMeters) {
    m_io.setElevatorPosition(setpointMeters);
  }

  public void runCharacterization(double outputVolts) {
    runElevatorOpenLoop(outputVolts);
  }

  public void stop() {
    m_io.stop();
  }

  private boolean isWithinUpperBound(double positionMeters) {
    return positionMeters < kElevatorUpperBoundMeters;
  }

  private boolean isWithinLowerBound(double positionMeters) {
    return positionMeters > kElevatorLowerBoundMeters;
  }

  public double getPositionMeters() {
    return m_inputs.elevatorPositionMeters;
  }

  public double getVelocityMetersPerSec() {
    return m_inputs.elevatorVelocityMetersPerSec;
  }

  public void setLevel(ElevatorLevel elevatorLevel) {
    m_elevatorLevel = elevatorLevel;
  }

  public ElevatorLevel getLevel() {
    return m_elevatorLevel;
  }

  public boolean isSetpointAchieved() {
    return Math.abs(m_inputs.elevatorErrorMeters) < kElevatorErrorToleranceMeters;
  }
}
