package frc.robot.subsystems.EndEffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private EndEffectorIO m_endEffector;
  private EndEffectorIOInputsAutoLogged m_inputs = new EndEffectorIOInputsAutoLogged();
  private Debouncer m_holdingGamePieceDebouncer = new Debouncer(0.5);

  private final Alert m_motorDisconnectedAlert;

  public EndEffector(EndEffectorIO endEffectorIO) {
    m_endEffector = endEffectorIO;

    m_motorDisconnectedAlert = new Alert("EndEffector motor disconnected!", AlertType.kError);
  }

  @Override
  public void periodic() {
    m_endEffector.updateInputs(m_inputs);
    Logger.processInputs("EndEffector", m_inputs);

    m_motorDisconnectedAlert.set(!m_inputs.endEffectorConnected);
  }

  public void runEndEffectorIntake() {
    m_endEffector.setVoltage(12.0);
  }

  public void runEndEffectorOuttake() {
    m_endEffector.setVoltage(-12.0);
  }

  public void stopEndEffector() {
    m_endEffector.setVoltage(0.0);
  }

  public void holdAlgae() {
    m_endEffector.setVoltage(12.0);
  }

  @AutoLogOutput
  public boolean isHoldingAlgae() {
    // needs to be tested, but high current with no spin should indicate holding algae
    return m_holdingGamePieceDebouncer.calculate(
        Math.abs(m_inputs.velocityRPM) < 20 && m_inputs.currentAmps > 10);
  }
}
