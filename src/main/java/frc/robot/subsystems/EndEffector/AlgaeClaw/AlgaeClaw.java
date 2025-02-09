package frc.robot.subsystems.EndEffector.AlgaeClaw;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw {

  private AlgaeClawIO m_algaeClawIO;
  private Alert m_algaeClawDisconnectedAlert;
  private AlgaeClawIOInputsAutoLogged m_algaeInputs;

  public AlgaeClaw(AlgaeClawIO io) {
    m_algaeClawIO = io;
    m_algaeClawDisconnectedAlert = new Alert("Algae claw motor disconnected!", AlertType.kError);
    m_algaeInputs = new AlgaeClawIOInputsAutoLogged();
  }

  public void periodic() {
    m_algaeClawIO.updateInputs(m_algaeInputs);
    Logger.processInputs("EndEffector/AlgaeClaw", m_algaeInputs);

    // Update alerts
    m_algaeClawDisconnectedAlert.set(!m_algaeInputs.algaeClawConnected);
  }

  public void runAlgaeClaw(double volt) {
    m_algaeClawIO.setVoltage(volt);
  }

  public void stop() {
    m_algaeClawIO.setVoltage(0);
  }
}
