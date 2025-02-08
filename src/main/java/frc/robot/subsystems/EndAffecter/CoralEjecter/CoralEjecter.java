package frc.robot.subsystems.EndAffecter.CoralEjecter;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class CoralEjecter {

  private Alert m_coralEjecterdisconnectedAlert;
  private CoralEjecterIO m_CoralEjecterIO;
  private CoralEjecterInputsAutoLogged m_inputs = new CoralEjecterInputsAutoLogged();

  public CoralEjecter(CoralEjecterIO io) {

    this.m_CoralEjecterIO = io;
    m_coralEjecterdisconnectedAlert = new Alert("CoralEjecter disconnected!", AlertType.kError);
  }

  public void periodic() {
    m_CoralEjecterIO.updateInputs(m_inputs);
    Logger.processInputs("Collector/CoralEjecter", m_inputs);

    // Update alerts
    m_coralEjecterdisconnectedAlert.set(!m_inputs.CoralEjecterConnected);
  }

  public void runCoralEjecter(double volts) {
    m_CoralEjecterIO.setVoltage(volts);
  }

  public void stop() {
    m_CoralEjecterIO.setVoltage(0.0);
  }
}
