package frc.robot.subsystems.EndEffector.AlgaeClaw;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.io.beambreak.BeamBreakIO;
import frc.robot.io.beambreak.BeamBreakIOInputsAutoLogged;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.util.GamePieces.GamePieceVisualizer;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class AlgaeClaw {
  private Alert m_algaeClawDisconnectedAlert;
  private AlgaeClawIO m_algaeClawIO;
  private AlgaeClawIOInputsAutoLogged m_algaeInputs = new AlgaeClawIOInputsAutoLogged();
  private final BeamBreakIO m_beamBreakIO;
  private final BeamBreakIOInputsAutoLogged m_beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  @Setter private boolean applyHoldVoltage = false;

  public AlgaeClaw(AlgaeClawIO io, BeamBreakIO beamBreakIO) {
    m_algaeClawIO = io;
    m_beamBreakIO = beamBreakIO;

    m_algaeClawDisconnectedAlert = new Alert("Algae claw motor disconnected!", AlertType.kError);
  }

  public void periodic() {
    m_algaeClawIO.updateInputs(m_algaeInputs);
    m_beamBreakIO.updateInputs(m_beamBreakInputs);
    Logger.processInputs("EndEffector/AlgaeClaw", m_algaeInputs);
    Logger.processInputs("EndEffector/AlgaeClaw/BeamBreak", m_beamBreakInputs);

    // Update alerts
    m_algaeClawDisconnectedAlert.set(!m_algaeInputs.algaeClawConnected);

    // Update game piece visualizer
    GamePieceVisualizer.setHasAlgae(m_beamBreakInputs.state);
    LEDSubsystem.getInstance().setAlgaeHeld(m_beamBreakInputs.state);

    if (applyHoldVoltage) {
      runAlgaeClaw(3);
      if (!isBeamBreakTriggered()) {
        stop();
        applyHoldVoltage = false;
      }
    }
  }

  public void runAlgaeClaw(double volt) {
    m_algaeClawIO.setVoltage(volt);
  }

  public BeamBreakIO getBeamBreakIO() {
    return m_beamBreakIO;
  }

  public boolean isBeamBreakTriggered() {
    return m_beamBreakInputs.state;
  }

  public void stop() {
    m_algaeClawIO.setVoltage(0);
  }
}
