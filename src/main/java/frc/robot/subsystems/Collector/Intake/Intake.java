package frc.robot.subsystems.Collector.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class Intake {
  private Alert motor1disconnectedAlert;
  private Alert motor2disconnectedAlert;
  private IntakeIO intakeIO;
  private IntakeIOInputsAutoLogged m_inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.intakeIO = io;

    motor1disconnectedAlert = new Alert("Intake motor1 disconnected!", AlertType.kError);
    motor2disconnectedAlert = new Alert("Intake motor2 disconnected!", AlertType.kError);
  }

  public void periodic() {
    intakeIO.updateInputs(m_inputs);
    Logger.processInputs("Collector/Intake", m_inputs);

    // Update alerts
    motor1disconnectedAlert.set(!m_inputs.intake1Connected);
    motor2disconnectedAlert.set(!m_inputs.intake2Connected);
  }

  public void runIntake(double volts) {
    intakeIO.setVoltage(volts);
  }

  public void stop() {
    intakeIO.setVoltage(0.0);
  }
}
