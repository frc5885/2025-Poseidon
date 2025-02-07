package frc.robot.subsystems.Collector.Feeder;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Feeder {
  private Alert motorDisconnectedAlert;
  private FeederIO feederIO;
  private FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.feederIO = io;

    motorDisconnectedAlert = new Alert("Feeder motor disconnected!", AlertType.kError);
  }

  public void periodic() {
    feederIO.updateInputs(m_inputs);
    Logger.processInputs("Collector/Feeder", m_inputs);

    // Update alert
    motorDisconnectedAlert.set(!m_inputs.feederConnected);
  }

  public void runFeeder(double volts) {
    feederIO.setVoltage(volts);
  }

  public void stop() {
    feederIO.setVoltage(0.0);
  }
}
