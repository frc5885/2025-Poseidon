package frc.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.io.beambreak.BeamBreakIO;
import frc.robot.io.beambreak.BeamBreakIOInputsAutoLogged;
import frc.robot.subsystems.Feeder.FeederConstants.*;
import frc.robot.util.GamePieces.GamePieceVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final Alert m_motor1DisconnectedAlert;
  private final Alert m_motor2DisconnectedAlert;
  private final FeederIO m_feederIO;
  private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();
  private final BeamBreakIO m_beamBreakIO;
  private final BeamBreakIOInputsAutoLogged m_beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  private FeederState m_state = FeederState.IDLE;

  private boolean m_isHandOffReady = false;

  public Feeder(FeederIO io, BeamBreakIO beamBreakIO) {
    m_feederIO = io;
    m_beamBreakIO = beamBreakIO;

    m_motor1DisconnectedAlert = new Alert("Feeder motor 1 disconnected!", AlertType.kError);
    m_motor2DisconnectedAlert = new Alert("Feeder motor 2 disconnected!", AlertType.kError);
  }

  public void periodic() {
    m_feederIO.updateInputs(m_inputs);
    Logger.processInputs("Feeder", m_inputs);

    m_beamBreakIO.updateInputs(m_beamBreakInputs);
    Logger.processInputs("Feeder/BeamBreak", m_beamBreakInputs);

    // Update alert
    m_motor1DisconnectedAlert.set(!m_inputs.motor1Connected);
    m_motor2DisconnectedAlert.set(!m_inputs.motor2Connected);

    switch (m_state) {
      case IDLE:
        stop();
        break;
      case FEEDING:
        runFeeder(6.0);
        if (isBeamBreakTriggered()) {
          m_state = FeederState.FEEDING_SLOW;
        }
        break;
      case FEEDING_SLOW:
        runFeeder(2.0);
        if (!isBeamBreakTriggered()) {
          m_state = FeederState.IDLE;
          m_isHandOffReady = true;
        }
        break;
    }

    Logger.recordOutput("Feeder/handoffReady", m_isHandOffReady);
  }

  public void runFeeder(double volts) {
    m_feederIO.setVoltage(volts);
  }

  public boolean isBeamBreakTriggered() {
    return m_beamBreakInputs.state;
  }

  public BeamBreakIO getBeamBreakIO() {
    return m_beamBreakIO;
  }

  public void stop() {
    m_feederIO.setVoltage(0.0);
  }

  // @AutoLogOutput
  public boolean getIsHandoffReady() {
    return m_isHandOffReady;
  }

  public Trigger getHandoffTrigger() {
    return new Trigger(() -> m_isHandOffReady);
  }

  public void handoffComplete() {
    m_isHandOffReady = false;

    if (Constants.kCurrentMode == Mode.SIM) {
      GamePieceVisualizer.setHasCoral(true);
    }
  }

  /** Only use this in simulation */
  public void setHandoffReady() {
    if (!GamePieceVisualizer.hasCoral()) {
      m_isHandOffReady = true;
    }
  }

  /** Set the state of the feeder, only when it is idle or feeding */
  public void setFeederState(FeederState state) {
    if (m_state == FeederState.FEEDING_SLOW || m_isHandOffReady) return;

    m_state = state;
  }

  @AutoLogOutput
  public FeederState getFeederState() {
    return m_state;
  }
}
