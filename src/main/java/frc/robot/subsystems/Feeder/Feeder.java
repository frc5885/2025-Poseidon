package frc.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.beambreak.BeamBreakIO;
import frc.robot.io.beambreak.BeamBreakIOInputsAutoLogged;
import frc.robot.subsystems.Feeder.FeederConstants.*;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final Alert m_motor1DisconnectedAlert;
  private final Alert m_motor2DisconnectedAlert;
  private final FeederIO m_feederIO;
  private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();
  private final BeamBreakIO m_beamBreakIO;
  private final BeamBreakIOInputsAutoLogged m_beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  private FeederState m_state = FeederState.IDLE;

  public Feeder(FeederIO io, BeamBreakIO beamBreakIO) {
    m_feederIO = io;
    m_beamBreakIO = beamBreakIO;

    m_motor1DisconnectedAlert = new Alert("Feeder motor 1 disconnected!", AlertType.kError);
    m_motor2DisconnectedAlert = new Alert("Feeder motor 2 disconnected!", AlertType.kError);
  }

  public void periodic() {
    m_feederIO.updateInputs(m_inputs);
    Logger.processInputs("Collector/Feeder", m_inputs);

    m_beamBreakIO.updateInputs(m_beamBreakInputs);
    Logger.processInputs("Collector/Intake/BeamBreak", m_beamBreakInputs);

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
        }
        break;
    }
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

  public boolean isCurrentOverThreshold() {
    return Arrays.stream(m_inputs.currentAmps).average().getAsDouble()
        > FeederConstants.kFeederCurrentThreshold;
  }

  public void setFeederState(FeederState state) {
    if (m_state == FeederState.FEEDING_SLOW) return;

    m_state = state;
  }

  public FeederState getFeederState() {
    return m_state;
  }
}
