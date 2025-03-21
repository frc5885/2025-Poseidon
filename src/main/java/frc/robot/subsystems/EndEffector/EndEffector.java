package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private EndEffectorIO m_endEffector;
  private EndEffectorIOInputsAutoLogged m_inputs = new EndEffectorIOInputsAutoLogged();

  public EndEffector(EndEffectorIO endEffectorIO) {
    m_endEffector = endEffectorIO;
  }

  @Override
  public void periodic() {
    m_endEffector.updateInputs(m_inputs);
    Logger.processInputs("EndEffector", m_inputs);
  }

  public void runEndEffector(double volts) {
    m_endEffector.setVoltage(volts);
  }

  public void stopEndEffector() {
    m_endEffector.setVoltage(0.0);
  }
}
