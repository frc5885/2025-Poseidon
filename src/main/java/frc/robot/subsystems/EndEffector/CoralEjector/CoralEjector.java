package frc.robot.subsystems.EndEffector.CoralEjector;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.io.beambreak.BeamBreakIO;
import frc.robot.io.beambreak.BeamBreakIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class CoralEjector {

  private Alert m_coralEjectorDisconnectedAlert;
  private CoralEjectorIO m_coralEjectorIO;
  private CoralEjectorIOInputsAutoLogged m_inputs = new CoralEjectorIOInputsAutoLogged();
  private BeamBreakIO m_beamBreakIO;
  private BeamBreakIOInputsAutoLogged m_beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  public CoralEjector(CoralEjectorIO io, BeamBreakIO beamBreakIO) {
    m_coralEjectorIO = io;
    m_coralEjectorDisconnectedAlert = new Alert("Coral ejector disconnected!", AlertType.kError);
    m_beamBreakIO = beamBreakIO;
  }

  public void periodic() {
    m_coralEjectorIO.updateInputs(m_inputs);
    Logger.processInputs("EndEffector/CoralEjector", m_inputs);

    m_beamBreakIO.updateInputs(m_beamBreakInputs);
    Logger.processInputs("EndEffector/CoralEjector/BeamBreak", m_beamBreakInputs);

    // Update alerts
    m_coralEjectorDisconnectedAlert.set(!m_inputs.coralEjectorConnected);
  }

  public void runCoralEjector(double volts) {
    m_coralEjectorIO.setVoltage(volts);
  }

  public void stop() {
    m_coralEjectorIO.setVoltage(0.0);
  }

  public BeamBreakIO getBeamBreakIO() {
    return m_beamBreakIO;
  }

  public boolean isBeamBreakTriggered() {
    return m_beamBreakInputs.state;
  }
}
