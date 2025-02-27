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
                DCMotor.getNeo550(1), 0.001, AlgaeClawConstants.kGearRatio),
            DCMotor.getNeo550(1));
  }

  public void updateInputs(AlgaeClawIOInputs inputs) {
    m_algaeClawFlywheelSim.update(0.020);

    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps = m_algaeClawFlywheelSim.getCurrentDrawAmps();
    inputs.velocityRPM = m_algaeClawFlywheelSim.getAngularVelocityRPM();
    inputs.positionRotations = 0.0;
    inputs.algaeClawConnected = true;
  }

  public void setVoltage(double volts) {
    m_appliedVolts = volts;
    m_algaeClawFlywheelSim.setInput(volts);
  }
}
