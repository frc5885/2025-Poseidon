package frc.robot.subsystems.EndAffecter.CoralEjecter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.EndAffecter.CoralEjecter.CoralEjecterIO.CoralEjecterInputs;
import frc.robot.subsystems.EndAffecter.EndAffecterConstant.CoralEjecterConstants;

public class coralEjecterIOSim implements CoralEjecterIO {

  FlywheelSim m_FlywheelSim;
  double m_appliedVolts;

  public coralEjecterIOSim() {
    m_FlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), 0.001, CoralEjecterConstants.CoralEjecterGearRatio),
            DCMotor.getNEO(1),
            0.0);
  }

  public void updateInputs(CoralEjecterInputs m_inputs) {
    m_FlywheelSim.update(0.020);

    m_inputs.appliedVolts = m_appliedVolts;
    m_inputs.currentAmps = m_FlywheelSim.getCurrentDrawAmps();
    m_inputs.velocityRPM = m_FlywheelSim.getAngularVelocityRPM();
    m_inputs.positionRotations = 0.0;
  }

  public void setVoltage(double volts) {
    m_appliedVolts = volts;
    m_FlywheelSim.setInputVoltage(volts);
  }
}
