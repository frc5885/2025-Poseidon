package frc.robot.subsystems.Collector.intake;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOsim {

  private FlywheelSim m_FlywheelSim;

  public IntakeIOsim() {
    m_FlywheelSim = new FlywheelSim(null, null, null);
  }
}
