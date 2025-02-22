// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDS;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem {
  private static LEDSubsystem instance;
  private static final AddressableLED m_leds;
  private static final AddressableLEDBuffer m_buffer;
  @Setter private static LEDStates states;
  private static final AddressableLEDBufferView m_left;
  private static final AddressableLEDBufferView m_right;
  private static final int kLength = 60;
  private static final int kLEDPort = 0;

  static {
    m_leds = new AddressableLED(kLEDPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_leds.setLength(kLength);
    m_leds.setData(m_buffer);
    m_leds.start();

    states = LEDStates.IDLE;
    m_left = m_buffer.createView(0, kLength / 2 - 1);
    m_right = m_buffer.createView(kLength / 2, kLength - 1);
  }

  public static LEDSubsystem getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
    }
    return instance;
  }

  public void periodic() {
    states.getPattern().applyTo(m_left);
    states.getPattern().applyTo(m_right);

    Logger.recordOutput("LEDColor", m_left.getLED(0).toHexString());
  }

  @RequiredArgsConstructor
  public static enum LEDStates {
    IDLE(Color.kWhite, LEDPattern.solid(Color.kWhite)),
    // DISABLED(Color.kBlue, null),
    INTAKE_RUNNING(Color.kYellow, LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.5))),
    INTAKE_RUNNING_SEES_PIECE(
        Color.kPurple, LEDPattern.solid(Color.kPurple).blink(Seconds.of(0.5))),
    CORAL_IN_FEEDER(Color.kYellow, LEDPattern.solid(Color.kYellow)),
    SCORING_LINE_UP(Color.kBlue, LEDPattern.solid(Color.kBlue)),
    SCORED(Color.kBlue, LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.5))),
    HOLDING_PIECE(Color.kGreen, LEDPattern.solid(Color.kGreen).breathe(Seconds.of(2.0))),
    RESETTING_SUPERSTRUCTURE(Color.kRed, LEDPattern.solid(Color.kRed).breathe(Seconds.of(2.0))),
  // AUTO(Color.kBlue, null)
  ;

    @Getter private final Color color;
    @Getter private final LEDPattern pattern;
  }
}
