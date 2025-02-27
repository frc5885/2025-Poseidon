package frc.robot.subsystems.SuperStructure.Wrist;

import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.ArmConstants.kArmStartingPositionRadians;
import static frc.robot.subsystems.SuperStructure.SuperStructureConstants.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.MovingFrameSingleJointedArmSim;
import java.util.function.DoubleSupplier;

public class WristIOSim implements WristIO {
  private final MovingFrameSingleJointedArmSim m_wristSim;

  private double m_appliedVolts;

  public WristIOSim() {
    m_wristSim =
        new MovingFrameSingleJointedArmSim(
            DCMotor.getNeo550(1),
            kWristMotorReduction,
            kWristMOI_kgm2,
            kWristLengthMeters,
            kWristMinAngleRads,
            kWristMaxAngleRads,
            true,
            kWristStartingPositionRadians,
            kArmStartingPositionRadians);
  }

  @Override
  public void updateInputs(WristIOInputs inputs, DoubleSupplier armAngleSupplier) {
    double armAngleRads = armAngleSupplier.getAsDouble();

    m_wristSim.setArmAngle(armAngleRads);
    m_wristSim.update(0.02);

    inputs.wristConnected = true;
    inputs.positionRads = m_wristSim.getAngleRads();
    inputs.realWorldPositionRads = inputs.positionRads + armAngleRads;
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
