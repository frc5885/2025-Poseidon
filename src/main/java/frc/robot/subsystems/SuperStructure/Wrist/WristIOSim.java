package frc.robot.subsystems.SuperStructure.Wrist;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.MovingFrameSingleJointedArmSim;
import java.util.function.DoubleSupplier;

public class WristIOSim implements WristIO {
  private final MovingFrameSingleJointedArmSim m_wristSim;
  private DoubleSupplier m_armAngleRads;

  private double m_appliedVolts;

  public WristIOSim(DoubleSupplier armAngleRads) {
    m_armAngleRads = armAngleRads;
    m_wristSim =
        new MovingFrameSingleJointedArmSim(
            DCMotor.getNEO(1),
            kWristMotorReduction,
            2.767,
            kWristLengthMeters,
            kWristMinAngleRads,
            kWristMaxAngleRads,
            true,
            kWristStartingPositionRadians,
            m_armAngleRads.getAsDouble());
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    m_wristSim.setArmAngle(m_armAngleRads.getAsDouble());
    m_wristSim.update(0.02);

    inputs.wristConnected = true;
    inputs.positionRads = m_wristSim.getAngleRads();
    inputs.realWorldPositionRads = inputs.positionRads + m_armAngleRads.getAsDouble();
    inputs.wristVelocityRadPerSec = m_wristSim.getVelocityRadPerSec();
    inputs.wristAppliedVolts = m_appliedVolts;
    inputs.wristCurrentAmps = m_wristSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double outputVolts) {
    m_appliedVolts = MathUtil.clamp(outputVolts, -12.0, 12.0);
    m_wristSim.setInputVoltage(m_appliedVolts);
  }
}
