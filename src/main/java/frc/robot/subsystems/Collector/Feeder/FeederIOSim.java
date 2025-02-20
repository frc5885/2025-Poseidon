package frc.robot.subsystems.Collector.Feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Collector.CollectorConstants.FeederConstants;

public class FeederIOSim implements FeederIO {

  private double m_appliedVolts;
  private FlywheelSim m_flywheelSim;

  public FeederIOSim() {
    m_flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), 0.001, FeederConstants.kGearRatio),
            DCMotor.getNEO(1));
  }

  public void updateInputs(FeederIOInputs inputs) {

    m_flywheelSim.update(0.020);

    inputs.positionRotations = 0.0;
    inputs.velocityRPM = m_flywheelSim.getAngularVelocityRPM();
    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps = m_flywheelSim.getCurrentDrawAmps();
    inputs.feederConnected = true;
  }

  public void setVoltage(double volts) {
    m_appliedVolts = volts;
    m_flywheelSim.setInput(volts);
  }
}
