// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDS;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;
import java.util.stream.IntStream;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem {
  private static LEDSubsystem instance;
  private final AddressableLED m_leds;
  private final AddressableLEDBuffer m_buffer;
  private LEDStates states;
  private final AddressableLEDBufferView m_view;
  private static final int kLength = 33;
  private static final int kLEDPort = 0;

  @Setter @Getter private boolean coralHeld = false;
  @Setter @Getter private boolean algaeHeld = false;
  @Setter private boolean seesGamePiece = false;
  @Setter private boolean isPhotonDied = false;

  private LEDSubsystem() {
    m_leds = new AddressableLED(kLEDPort);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_leds.setLength(kLength);
    m_leds.setData(m_buffer);
    m_leds.start();

    states = LEDStates.IDLE;
    m_view = m_buffer.createView(0, kLength - 1);
  }

  public static synchronized LEDSubsystem getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
    }
    return instance;
  }

  public void setStates(LEDStates newState) {
    if (!DriverStation.isAutonomous() || newState == LEDStates.AUTO) {
      states = newState;
      if (newState == LEDStates.IDLE && (coralHeld || algaeHeld)) {
        states = LEDStates.HOLDING_PIECE;
      } else {
        states = newState;
      }
    }
  }

  public void periodic() {
    states.getPattern().applyTo(m_view);

    if (isPhotonDied) {
      states = LEDStates.PHOTON_DIED;
    } else {
      if (states == LEDStates.INTAKE_RUNNING && seesGamePiece) {
        setStates(LEDStates.INTAKE_RUNNING_SEES_PIECE);
      } else if (states == LEDStates.INTAKE_RUNNING_SEES_PIECE && !seesGamePiece) {
        setStates(LEDStates.INTAKE_RUNNING);
      }

      if (states == LEDStates.SCORED && (!coralHeld || !algaeHeld)) {
        states = LEDStates.RESETTING_SUPERSTRUCTURE;
      }
    }

    Logger.recordOutput(
        "LED",
        IntStream.range(0, kLength / 2)
            .mapToObj(i -> m_view.getLED(i).toHexString())
            .toArray(String[]::new));

    m_leds.setData(m_buffer);
  }

  @RequiredArgsConstructor
  public static enum LEDStates {
    IDLE(
        LEDPattern.solid((Color.kWhite))
            .mask(
                LEDPattern.steps(Map.of(0.5, Color.kWhite))
                    .scrollAtRelativeSpeed(Percent.per(Second).of(25.0)))),
    PHOTON_DIED(
        LEDPattern.solid((Color.kRed))
            .mask(
                LEDPattern.steps(Map.of(0.5, Color.kRed))
                    .scrollAtRelativeSpeed(Percent.per(Second).of(50.0)))),
    DISABLED(LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(25))),
    INTAKE_RUNNING(LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.1))),
    INTAKE_RUNNING_SEES_PIECE(LEDPattern.solid(Color.kPurple).blink(Seconds.of(0.1))),
    CORAL_IN_FEEDER(LEDPattern.solid(Color.kYellow)),
    ALGAE_INTAKE_LINE_UP(LEDPattern.solid(Color.kCyan)),
    SCORING_LINE_UP(LEDPattern.solid(Color.kBlue)),
    SCORED(LEDPattern.solid(Color.kBlue).blink(Seconds.of(0.05))),
    HOLDING_PIECE(LEDPattern.solid(Color.kGreen).breathe(Seconds.of(2.0))),
    RESETTING_SUPERSTRUCTURE(LEDPattern.solid(Color.kRed).breathe(Seconds.of(2.0))),
    AUTO(
        LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kYellow, Color.kGreen)
            .mask(LEDPattern.progressMaskLayer(() -> Timer.getMatchTime() / 15.0)));

    @Getter private final LEDPattern pattern;
  }
}
