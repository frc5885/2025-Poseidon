package frc.robot.subsystems.Collector.Intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.io.beambreak.BeamBreakIO;
import frc.robot.io.beambreak.BeamBreakIOInputsAutoLogged;
import org.ironmaple.simulation.IntakeSimulation;
import org.littletonrobotics.junction.Logger;

public class Intake {
  private final Alert m_motor1DisconnectedAlert;
  private final Alert m_motor2DisconnectedAlert;
  private final IntakeIO m_intakeIO;
  private final IntakeIOInputsAutoLogged m_intakeInputs = new IntakeIOInputsAutoLogged();

  // track state internally
  private boolean m_isIntakeExtended = false;

  public Intake(IntakeIO io) {
    m_intakeIO = io;

    m_motor1DisconnectedAlert = new Alert("Intake Motor1 disconnected!", AlertType.kError);
    m_motor2DisconnectedAlert = new Alert("Intake Motor2 disconnected!", AlertType.kError);
  }

  public void periodic() {
    m_intakeIO.updateInputs(m_intakeInputs);
    Logger.processInputs("Collector/Intake", m_intakeInputs);

    // Update alerts
    m_motor1DisconnectedAlert.set(!m_intakeInputs.motor1Connected);
    m_motor2DisconnectedAlert.set(!m_intakeInputs.motor2Connected);

    // Update game piece visualizer
    // GamePieceVisualizer.setHasCoral(m_beamBreakInputs.state);
    // LEDSubsystem.getInstance().setHasGamePiece(m_beamBreakInputs.state);
  }

  public void runIntake(double volts) {
    m_intakeIO.setVoltage(volts);
  }

  public void stop() {
    m_intakeIO.setVoltage(0.0);
  }

  public boolean isExtended() {
    return m_isIntakeExtended;
  }

  public void extend() {
    m_intakeIO.extendIntake();
    m_isIntakeExtended = true;
  }

  public void retract() {
    m_intakeIO.retractIntake();
    m_isIntakeExtended = false;
  }

  public IntakeSimulation getMapleIntakeSim() {
    return ((IntakeIOSim) m_intakeIO).getMapleIntakeSimulation();
  }
}
