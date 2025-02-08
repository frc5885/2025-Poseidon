package frc.robot.subsystems.EndEffector.AlgaeClaw;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.AlgaeClawConstants;

public class AlgaeClawIOSim implements AlgaeClawIO {
  private FlywheelSim m_algaeClawFlywheelSim;
  private double m_appliedVolts;

  public AlgaeClawIOSim() {

    m_algaeClawFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), 0.001, AlgaeClawConstants.AlgaeClawGearRatio),
            DCMotor.getNEO(1),
            0.0);
  }

  public void updateInputs(AlgaeClawIOIntputs m_inputs) {
    m_algaeClawFlywheelSim.update(0.020);

    m_inputs.appliedVolts = m_appliedVolts;
    m_inputs.currentAmps = m_algaeClawFlywheelSim.getCurrentDrawAmps();
    m_inputs.velocityRPM = m_algaeClawFlywheelSim.getAngularVelocityRPM();
    m_inputs.positionRotations = 0.0;
    m_inputs.algaeClawConnected = true;
  }

  public void setVoltage(double volts) {
    m_appliedVolts = volts;
    m_algaeClawFlywheelSim.setInput(volts);
  }
}
