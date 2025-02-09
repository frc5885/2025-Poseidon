package frc.robot.subsystems.Collector.Feeder;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Feeder {
  private Alert m_motorDisconnectedAlert;
  private FeederIO m_feederIO;
  private FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.m_feederIO = io;

    m_motorDisconnectedAlert = new Alert("Feeder motor disconnected!", AlertType.kError);
  }

  public void periodic() {
    m_feederIO.updateInputs(m_inputs);
    Logger.processInputs("Collector/Feeder", m_inputs);

    // Update alert
    m_motorDisconnectedAlert.set(!m_inputs.feederConnected);
  }

  public void runFeeder(double volts) {
    m_feederIO.setVoltage(volts);
  }

  public void stop() {
    m_feederIO.setVoltage(0.0);
  }
}
