package frc.robot.subsystems.SuperStructure.Arm;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim m_armSim;

  private double m_appliedVolts;

  public ArmIOSim() {
    m_armSim =
        new SingleJointedArmSim(
            LinearSystemId.identifyPositionSystem(kArmKv, kArmKa),
            DCMotor.getNEO(1),
            kArmMotorReduction,
            kArmLengthMeters,
            kArmMinAngleRads,
            kArmMaxAngleRads,
            true,
            kArmStartingPositionRads);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    m_armSim.update(0.02);
    inputs.motorConnected = true;
    inputs.positionRads = m_armSim.getAngleRads();
    inputs.velocityRadsPerSec = m_armSim.getVelocityRadPerSec();
    inputs.appliedVolts = m_appliedVolts;
    inputs.currentAmps = m_armSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double outputVolts) {
    m_appliedVolts = MathUtil.clamp(outputVolts, -12.0, 12.0);
    m_armSim.setInputVoltage(m_appliedVolts);
  }
}
