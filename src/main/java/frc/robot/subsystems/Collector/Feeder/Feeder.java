package frc.robot.subsystems.Collector.Feeder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.beambreak.BeamBreakIO;
import frc.robot.io.beambreak.BeamBreakIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final Alert m_motor1DisconnectedAlert;
  private final Alert m_motor2DisconnectedAlert;
  private final FeederIO m_feederIO;
  private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();
  private final BeamBreakIO m_beamBreakIO;
  private final BeamBreakIOInputsAutoLogged m_beamBreakInputs = new BeamBreakIOInputsAutoLogged();

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
}
