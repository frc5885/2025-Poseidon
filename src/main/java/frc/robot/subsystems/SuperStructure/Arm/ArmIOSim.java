package frc.robot.subsystems.SuperStructure.Arm;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim m_armSim;

  private double m_appliedVolts;

  public ArmIOSim() {
    m_armSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            kArmMotorReduction,
            0.4,
            kArmLengthMeters,
            kArmMinAngleRads,
            kArmMaxAngleRads,
            true,
            kArmStartingPositionRadians);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    m_armSim.update(0.02);
    inputs.armConnected = true;
    // inputs.absolutePositionRads = m_armSim.getAngleRads();
    inputs.positionRads = m_armSim.getAngleRads();
    inputs.armVelocityRadPerSec = m_armSim.getVelocityRadPerSec();
    inputs.armAppliedVolts = m_appliedVolts;
    inputs.armCurrentAmps = m_armSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double outputVolts) {
    m_appliedVolts = MathUtil.clamp(outputVolts, -12.0, 12.0);
    m_armSim.setInputVoltage(m_appliedVolts);
  }
}
