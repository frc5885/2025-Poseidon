package frc.robot.subsystems.Collector.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Collector.CollectorConstants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

  private double m_appliedVolts;
  private FlywheelSim m_flywheelSim;

  public IntakeIOSim() {
    m_flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNEO(2), 0.001, IntakeConstants.kGearRatio),
            DCMotor.getNEO(2));
  }

  public void updateInputs(IntakeIOInputs inputs) {

    m_flywheelSim.update(0.020);

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
