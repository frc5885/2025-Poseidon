package frc.robot.subsystems.EndEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class EndEffectorIOSim implements EndEffectorIO {
  private FlywheelSim m_EndEffectorFlywheelSim;
  private double m_appliedVolts;

  public EndEffectorIOSim() {
    m_EndEffectorFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeo550(1), 0.001, EndEffectorConstants.kGearRatio),
            DCMotor.getNeo550(1));
  }

  public void updateInputs(EndEffectorIOInputs inputs) {
    m_EndEffectorFlywheelSim.update(0.020);

    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps = m_EndEffectorFlywheelSim.getCurrentDrawAmps();
    inputs.positionRotations = 0.0;
    inputs.velocityRPM = m_EndEffectorFlywheelSim.getAngularVelocityRPM();
    inputs.endEffectorConnected = true;
  }

  public void setVoltage(double volts) {
    m_appliedVolts = volts;
    m_EndEffectorFlywheelSim.setInput(volts);
  }
}
