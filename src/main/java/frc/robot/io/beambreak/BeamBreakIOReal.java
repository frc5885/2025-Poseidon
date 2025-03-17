package frc.robot.io.beambreak;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIOReal implements BeamBreakIO {
  private DigitalInput m_beamBreak;

  private Debouncer m_beamBreakDebouncer = new Debouncer(0.1);

  public BeamBreakIOReal(int beamBreakId) {
    m_beamBreak = new DigitalInput(beamBreakId);
  }

  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.state = m_beamBreakDebouncer.calculate(!m_beamBreak.get());
  }
}
