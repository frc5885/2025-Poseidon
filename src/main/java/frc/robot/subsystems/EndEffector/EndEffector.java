package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.beambreak.BeamBreakIO;
import frc.robot.subsystems.EndEffector.AlgaeClaw.AlgaeClaw;
import frc.robot.subsystems.EndEffector.AlgaeClaw.AlgaeClawIO;
import frc.robot.subsystems.EndEffector.CoralEjector.CoralEjector;
import frc.robot.subsystems.EndEffector.CoralEjector.CoralEjectorIO;

public class EndEffector extends SubsystemBase {
  private AlgaeClaw m_algaeClaw;
  private CoralEjector m_coralEjector;

  public EndEffector(
      AlgaeClawIO algaeClawIO, CoralEjectorIO coralEjectorIO, BeamBreakIO beamBreakIO) {
    m_algaeClaw = new AlgaeClaw(algaeClawIO, beamBreakIO);
    m_coralEjector = new CoralEjector(coralEjectorIO);
  }

  @Override
  public void periodic() {
    m_algaeClaw.periodic();
    m_coralEjector.periodic();
  }

  public void runCoralEjector(double volts) {
    m_coralEjector.runCoralEjector(volts);
  }

  public void runAlgaeClaw(double volts) {
    m_algaeClaw.runAlgaeClaw(volts);
  }

  public boolean isAlgaeHeld() {
    return m_algaeClaw.isBeamBreakTriggered();
  }

  public BeamBreakIO getBeamBreakIO() {
    return m_algaeClaw.getBeamBreakIO();
  }

  public void stopCoralEjector() {
    m_coralEjector.stop();
  }

  public void stopAlgaeClaw() {
    m_algaeClaw.stop();
  }
}
