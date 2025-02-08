package frc.robot.subsystems.EndAffecter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.EndAffecter.AlgaeClaw.AlgaeClaw;
import frc.robot.subsystems.EndAffecter.AlgaeClaw.AlgaeClawIO;
import frc.robot.subsystems.EndAffecter.CoralEjecter.CoralEjecter;
import frc.robot.subsystems.EndAffecter.CoralEjecter.CoralEjecterIO;

public class EndAffecter extends SubsystemBase {
  private AlgaeClaw m_algaeClaw;
  private CoralEjecter m_coralEjecter;

  public EndAffecter(AlgaeClawIO algaeClawIO, CoralEjecterIO coralEjecterIO) {
    m_algaeClaw = new AlgaeClaw(algaeClawIO);
    m_coralEjecter = new CoralEjecter(coralEjecterIO);
  }

  @Override
  public void periodic() {
    m_algaeClaw.periodic();
    m_coralEjecter.periodic();
  }

  public void runCoralEjecter(double volts) {
    m_coralEjecter.runCoralEjecter(volts);
  }

  public void stopCoralEjecter() {
    m_coralEjecter.stop();
  }

  public void runAlgaeClaw(double volts) {
    m_algaeClaw.runAlgaeClaw(volts);
  }

  public void stopAlgaeClaw() {
    m_algaeClaw.stop();
  }
}
