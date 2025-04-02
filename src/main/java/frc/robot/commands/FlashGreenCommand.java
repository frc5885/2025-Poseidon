package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;

public class FlashGreenCommand extends Command {
  private final LEDSubsystem m_ledSubsystem;
  private double startTime;
  private boolean switchedToRed;

  public FlashGreenCommand(LEDSubsystem ledSubsystem) {
    m_ledSubsystem = ledSubsystem;
  }

  @Override
  public void initialize() {
    m_ledSubsystem.setSquareStates(LEDStates.GREEN); // Green
    startTime = Timer.getFPGATimestamp();
    switchedToRed = false;
  }

  @Override
  public void execute() {
    // Wait 0.5 seconds before switching to red
    if (!switchedToRed && Timer.getFPGATimestamp() - startTime >= 0.5) {
      m_ledSubsystem.setSquareStates(LEDStates.RED); // Red
      switchedToRed = true;
    }
  }

  @Override
  public boolean isFinished() {
    return switchedToRed; // End command after switching to red
  }
}
