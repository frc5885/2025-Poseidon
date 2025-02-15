package frc.robot.io.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIOReal implements BeamBreakIO {
  private DigitalInput m_beamBreak;

  public BeamBreakIOReal(int beamBreakId) {
    m_beamBreak = new DigitalInput(beamBreakId);
  }

  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.state = m_beamBreak.get();
  }
}
