package frc.robot.subsystems.EndEffector;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private EndEffectorIO m_endEffector;
  private EndEffectorIOInputsAutoLogged m_inputs = new EndEffectorIOInputsAutoLogged();
  private Debouncer m_holdingGamePieceDebouncer = new Debouncer(0.5);

  private final LinearFilter m_currentFilter = LinearFilter.singlePoleIIR(0.3, 0.02);
  private final LinearFilter m_velocityFilter = LinearFilter.singlePoleIIR(0.05, 0.02);
  private double m_filteredVelocity = 0.0;
  private double m_filteredCurrent = 0.0;

  private final Alert m_motorDisconnectedAlert;

  public EndEffector(EndEffectorIO endEffectorIO) {
    m_endEffector = endEffectorIO;

    m_motorDisconnectedAlert = new Alert("EndEffector motor disconnected!", AlertType.kError);
  }

  @Override
  public void periodic() {
    m_endEffector.updateInputs(m_inputs);
    Logger.processInputs("EndEffector", m_inputs);

    m_motorDisconnectedAlert.set(!m_inputs.endEffectorConnected);

    m_filteredVelocity = m_velocityFilter.calculate(m_inputs.velocityRPM);
    m_filteredCurrent = m_currentFilter.calculate(m_inputs.currentAmps);

    Logger.recordOutput("EndEffector/FilteredVelocity", m_filteredVelocity);
    Logger.recordOutput("EndEffector/FilteredCurrent", m_filteredCurrent);
  }

  public void runEndEffectorIntake() {
    m_endEffector.setVoltage(12.0);
  }

  public void runEndEffectorOuttake() {
    m_endEffector.setVoltage(-10.0);
  }

  public void ejectAlgae() {
    m_endEffector.setVoltage(-12.0);
  }

  public void stopEndEffector() {
    m_endEffector.setVoltage(0.0);
  }

  public void holdAlgae() {
    m_endEffector.setVoltage(12.0);
  }

  @AutoLogOutput
  public boolean isHoldingCoral() {
    return m_filteredCurrent > 10 && m_filteredVelocity > 800;
  }

  @AutoLogOutput
  public boolean isHoldingAlgae() {
    // needs to be tested, but high current with no spin should indicate holding algae
    return m_filteredVelocity < 2000 && m_filteredCurrent > 8;
  }
}
