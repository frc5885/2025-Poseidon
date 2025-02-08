package frc.robot.subsystems.EndEffector.CoralEjector;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class CoralEjector {

  private Alert m_coralEjectordisconnectedAlert;
  private CoralEjectorIO m_CoralEjectorIO;
  private CoralEjectorInputsAutoLogged m_inputs = new CoralEjectorInputsAutoLogged();

  public CoralEjector(CoralEjectorIO io) {

    this.m_CoralEjectorIO = io;
    m_coralEjectordisconnectedAlert = new Alert("Coral ejector disconnected!", AlertType.kError);
  }

  public void periodic() {
    m_CoralEjectorIO.updateInputs(m_inputs);
    Logger.processInputs("EndEffector/CoralEjector", m_inputs);

    // Update alerts
    m_coralEjectordisconnectedAlert.set(!m_inputs.coralEjectorConnected);
  }

  public void runCoralEjector(double volts) {
    m_CoralEjectorIO.setVoltage(volts);
  }

  public void stop() {
    m_CoralEjectorIO.setVoltage(0.0);
  }
}
