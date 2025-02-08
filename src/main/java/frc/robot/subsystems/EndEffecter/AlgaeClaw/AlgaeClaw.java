package frc.robot.subsystems.EndEffecter.AlgaeClaw;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw {

  private AlgaeClawIO m_AlgaeClawIO;
  private Alert m_AlgaeClawDesconnectedAlert;
  private AlgaeClawIOIntputsAutoLogged m_algaeInputs;

  public AlgaeClaw(AlgaeClawIO io) {

    this.m_AlgaeClawIO = io;

    m_AlgaeClawDesconnectedAlert = new Alert("AlgaeClaw motor disconected!", AlertType.kError);
    m_algaeInputs = new AlgaeClawIOIntputsAutoLogged();
  }

  public void periodic() {

    m_AlgaeClawIO.updateInputs(m_algaeInputs);
    Logger.processInputs("Collector/algaeClawInputs", m_algaeInputs);

    // Update alerts
    m_AlgaeClawDesconnectedAlert.set(!m_algaeInputs.AlfeaClawConnected);
  }

  public void runAlgaeClaw(double volt) {
    m_AlgaeClawIO.setVoltage(volt);
  }

  public void stop() {
    m_AlgaeClawIO.setVoltage(0);
  }
}
