package frc.robot.subsystems.EndEffector.CoralEjector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.EndEffector.EndEffectorConstants.CoralEjectorConstants;

public class CoralEjectorIOSim implements CoralEjectorIO {

  FlywheelSim m_coralEjectorlywheelSim;
  double m_appliedVolts;

  public CoralEjectorIOSim() {
    m_coralEjectorlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), 0.001, CoralEjectorConstants.CoralEjectorGearRatio),
            DCMotor.getNEO(1),
            0.0);
  }

  public void updateInputs(CoralEjectorInputs m_inputs) {
    m_coralEjectorlywheelSim.update(0.020);

    m_inputs.appliedVolts = m_appliedVolts;
    m_inputs.currentAmps = m_coralEjectorlywheelSim.getCurrentDrawAmps();
    m_inputs.velocityRPM = m_coralEjectorlywheelSim.getAngularVelocityRPM();
    m_inputs.positionRotations = 0.0;
    m_inputs.coralEjectorConnected = true;
  }

  public void setVoltage(double volts) {
    m_appliedVolts = volts;
    m_coralEjectorlywheelSim.setInputVoltage(volts);
  }
}
