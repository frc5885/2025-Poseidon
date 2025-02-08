package frc.robot.subsystems.EndAffecter.AlgaeClaw;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.EndAffecter.EndAffecterConstant.AlgaeClawConstants;

public class AlgaeClawIOSim implements AlgaeClawIO {
  private FlywheelSim m_algaeClaFlywheelSim;
  private double m_algaeAppliedVolts;

  public AlgaeClawIOSim() {

    m_algaeClaFlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(1), 0.001, AlgaeClawConstants.AlgaeClawGearRatio),
            DCMotor.getNEO(1),
            0.0);
  }

  public void updateInputs(AlgaeClawIOIntputs m_inputs) {
    m_algaeClaFlywheelSim.update(0.020);

    m_inputs.appliedVolts = m_algaeAppliedVolts;
    m_inputs.currentAmps = m_algaeClaFlywheelSim.getCurrentDrawAmps();
    m_inputs.velocityRPM = m_algaeClaFlywheelSim.getAngularVelocityRPM();
    m_inputs.positionRotations = 0.0;
  }

  public void setVoltage(double volts) {
    m_algaeAppliedVolts = volts;
    m_algaeClaFlywheelSim.setInput(volts);
  }
}
