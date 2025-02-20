package frc.robot.subsystems.EndEffector.CoralEjector;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class CoralEjector {

  private Alert m_coralEjectorDisconnectedAlert;
  private CoralEjectorIO m_coralEjectorIO;
  private CoralEjectorIOInputsAutoLogged m_inputs = new CoralEjectorIOInputsAutoLogged();

  public CoralEjector(CoralEjectorIO io) {
    m_coralEjectorIO = io;
    m_coralEjectorDisconnectedAlert = new Alert("Coral ejector disconnected!", AlertType.kError);
  }

  public void periodic() {
    m_coralEjectorIO.updateInputs(m_inputs);
    Logger.processInputs("EndEffector/CoralEjector", m_inputs);

    // Update alerts
    m_coralEjectorDisconnectedAlert.set(!m_inputs.coralEjectorConnected);
  }

  public void runCoralEjector(double volts) {
    m_coralEjectorIO.setVoltage(volts);
  }

  public void stop() {
    m_coralEjectorIO.setVoltage(0.0);
  }
}
