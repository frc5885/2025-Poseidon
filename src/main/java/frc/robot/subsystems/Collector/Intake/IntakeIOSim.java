package frc.robot.subsystems.Collector.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Collector.CollectorConstants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

  private double m_appliedVolts;
  private FlywheelSim m_FlywheelSim;

  public IntakeIOSim() {
    m_FlywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(2), 0.001, IntakeConstants.intakeGearRatio),
            DCMotor.getNEO(2),
            0.0);
  }

  public void updateInputs(IntakeIOInputs inputs) {

    m_FlywheelSim.update(0.020);

    inputs.positionRotations = 0.0;
    inputs.velocityRPM = m_FlywheelSim.getAngularVelocityRPM();
    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps =
        new double[] {m_FlywheelSim.getCurrentDrawAmps(), m_FlywheelSim.getCurrentDrawAmps()};
  }

  public void setVoltage(double volts) {
    m_appliedVolts = volts;
    m_FlywheelSim.setInput(volts);
  }
}
