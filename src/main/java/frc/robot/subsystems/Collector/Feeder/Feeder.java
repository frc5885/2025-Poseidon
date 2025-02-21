package frc.robot.subsystems.Collector.Feeder;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Feeder {
  private final Alert m_motor1DisconnectedAlert;
  private final Alert m_motor2DisconnectedAlert;
  private final FeederIO m_feederIO;
  private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    m_feederIO = io;

    m_motor1DisconnectedAlert = new Alert("Feeder motor 1 disconnected!", AlertType.kError);
    m_motor2DisconnectedAlert = new Alert("Feeder motor 2 disconnected!", AlertType.kError);
  }

  public void periodic() {
    m_feederIO.updateInputs(m_inputs);
    Logger.processInputs("Collector/Feeder", m_inputs);

    // Update alert
    m_motor1DisconnectedAlert.set(!m_inputs.motor1Connected);
    m_motor2DisconnectedAlert.set(!m_inputs.motor2Connected);
  }

  public void runFeeder(double volts) {
    m_feederIO.setVoltage(volts);
  }

  public void stop() {
    m_feederIO.setVoltage(0.0);
  }
}
