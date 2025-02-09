package frc.robot.subsystems.Collector.Intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Intake {
  private final Alert m_motor1DisconnectedAlert;
  private final Alert m_motor2DisconnectedAlert;
  private final IntakeIO m_intakeIO;
  private final IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    m_intakeIO = io;

    m_motor1DisconnectedAlert = new Alert("Intake Motor1 disconnected!", AlertType.kError);
    m_motor2DisconnectedAlert = new Alert("Intake Motor2 disconnected!", AlertType.kError);
  }

  public void periodic() {
    m_intakeIO.updateInputs(m_inputs);
    Logger.processInputs("Collector/Intake", m_inputs);

    // Update alerts
    m_motor1DisconnectedAlert.set(!m_inputs.motor1Connected);
    m_motor2DisconnectedAlert.set(!m_inputs.motor2Connected);
  }

  public void runIntake(double volts) {
    m_intakeIO.setVoltage(volts);
  }

  public void stop() {
    m_intakeIO.setVoltage(0.0);
  }
}
