package frc.robot.subsystems.simplemanipulator.elevator;

import static frc.robot.subsystems.simplemanipulator.ManipulatorConstants.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

public class Elevator {
  private final ElevatorIO m_io;
  private final ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    m_io = io;
  }

  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("SimpleManipulator/Elevator", m_inputs);
  }

  public void runElevatorOpenLoop(double outputVolts) {
    // TODO MUST match the real implementation!
    if (outputVolts > 0) {
      m_io.setElevatorOpenLoop(isWithinUpperBound(getPosition()) ? outputVolts : 0.0);
    } else if (outputVolts < 0) {
      m_io.setElevatorOpenLoop(isWithinLowerBound(getPosition()) ? outputVolts : 0.0);
    } else {
      m_io.setElevatorOpenLoop(outputVolts);
    }
  }

  public void runElevatorSetpoint(double positionSetpoint) {
    m_io.setElevatorPosition(positionSetpoint);
  }

  public void stop() {
    m_io.stop();
  }

  public boolean isWithinUpperBound(double position) {
    return position < kElevatorUpperBound;
  }

  public boolean isWithinLowerBound(double position) {
    return position > kElevatorLowerBound;
  }

  public double getPosition() {
    return m_inputs.elevatorPosition;
  }

  public double getVelocity() {
    return m_inputs.elevatorVelocity;
  }
}
