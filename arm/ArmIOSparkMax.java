// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.WristConstants;

public class ArmIOSparkMax implements ArmIO {
  private final CANSparkMax m_motor =
      new CANSparkMax(WristConstants.kMotorPort, MotorType.kBrushless);
  private final SparkAbsoluteEncoder m_absoluteEncoder =
      m_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  public ArmIOSparkMax() {
    System.out.println("[Init] CreatingArmIOSparkMax");

    // m_motor.restoreFactoryDefaults();

    m_motor.setCANTimeout(250);

    m_motor.setInverted(false);

    m_motor.enableVoltageCompensation(12.0);
    m_motor.setSmartCurrentLimit(WristConstants.kCurrentLimit);

    // m_motor.burnFlash();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.absolutePositionRad = Units.rotationsToRadians(m_absoluteEncoder.getPosition());
    inputs.internalPositionRad = Units.rotationsToRadians(m_motor.getEncoder().getPosition());
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(m_motor.getEncoder().getVelocity());
    inputs.appliedVolts = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    inputs.currentAmps = m_motor.getOutputCurrent();
    inputs.tempCelsius = m_motor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    m_motor.setVoltage(volts);
  }
}
