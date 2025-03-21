package frc.robot.subsystems.Feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim implements FeederIO {

  private double m_appliedVolts;
  private FlywheelSim m_flywheelSim;

  public FeederIOSim() {
    m_flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeo550(1), 0.001, FeederConstants.kGearRatio),
            DCMotor.getNeo550(1));
  }

  public void updateInputs(FeederIOInputs inputs) {
    m_flywheelSim.update(0.02);

    inputs.positionRotations = 0.0;
    inputs.velocityRPM = m_flywheelSim.getAngularVelocityRPM();
    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps =
        new double[] {m_flywheelSim.getCurrentDrawAmps(), m_flywheelSim.getCurrentDrawAmps()};
    inputs.motor1Connected = true;
    inputs.motor2Connected = true;
  }

  public void setVoltage(double volts) {
    m_appliedVolts = volts;
    m_flywheelSim.setInput(volts);
  }
}
