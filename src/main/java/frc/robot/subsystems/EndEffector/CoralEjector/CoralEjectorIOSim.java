package frc.robot.subsystems.EndEffector.CoralEjector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.CoralEjectorConstants;

public class CoralEjectorIOSim implements CoralEjectorIO {

  private FlywheelSim m_coralEjectorFlywheelSim;
  private double m_appliedVolts;

  public CoralEjectorIOSim() {
    m_coralEjectorFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), 0.001, CoralEjectorConstants.kGearRatio),
            DCMotor.getNEO(1),
            0.0);
  }

  public void updateInputs(CoralEjectorIOInputs inputs) {
    m_coralEjectorFlywheelSim.update(0.020);

    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps = m_coralEjectorFlywheelSim.getCurrentDrawAmps();
    inputs.velocityRPM = m_coralEjectorFlywheelSim.getAngularVelocityRPM();
    inputs.positionRotations = 0.0;
    inputs.coralEjectorConnected = true;
  }

  public void setVoltage(double volts) {
    m_appliedVolts = volts;
    m_coralEjectorFlywheelSim.setInputVoltage(volts);
  }
}
