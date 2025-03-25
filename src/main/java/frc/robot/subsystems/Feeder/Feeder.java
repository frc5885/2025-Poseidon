package frc.robot.subsystems.Feeder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.io.beambreak.BeamBreakIO;
import frc.robot.io.beambreak.BeamBreakIOInputsAutoLogged;
import frc.robot.subsystems.LEDS.LEDSubsystem;
import frc.robot.subsystems.LEDS.LEDSubsystem.LEDStates;
import frc.robot.util.GamePieces.GamePieceVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final Alert m_motor1DisconnectedAlert;
  private final Alert m_motor2DisconnectedAlert;
  private final FeederIO m_feederIO;
  private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();
  private final BeamBreakIO m_beamBreakIO;
  private final BeamBreakIOInputsAutoLogged m_beamBreakInputs = new BeamBreakIOInputsAutoLogged();
  private final Debouncer m_beambreakDebouncer = new Debouncer(0.1);

  private boolean m_isHandOffReady = false;

  public Feeder(FeederIO io, BeamBreakIO beamBreakIO) {
    m_feederIO = io;
    m_beamBreakIO = beamBreakIO;

    m_motor1DisconnectedAlert = new Alert("Feeder motor 1 disconnected!", AlertType.kError);
    m_motor2DisconnectedAlert = new Alert("Feeder motor 2 disconnected!", AlertType.kError);
  }

  public void periodic() {
    m_feederIO.updateInputs(m_inputs);
    Logger.processInputs("Feeder", m_inputs);

    m_beamBreakIO.updateInputs(m_beamBreakInputs);
    Logger.processInputs("Feeder/BeamBreak", m_beamBreakInputs);

    // Update alert
    m_motor1DisconnectedAlert.set(!m_inputs.motor1Connected);
    m_motor2DisconnectedAlert.set(!m_inputs.motor2Connected);
  }

  public void runFeeder(double volts) {
    m_feederIO.setVoltage(volts);
  }

  public boolean isBeamBreakTriggered() {
    return m_beambreakDebouncer.calculate(m_beamBreakInputs.state);
  }

  public BeamBreakIO getBeamBreakIO() {
    return m_beamBreakIO;
  }

  public void stop() {
    m_feederIO.setVoltage(0.0);
  }

  @AutoLogOutput
  public boolean getIsHandoffReady() {
    return m_isHandOffReady;
  }

  public Trigger getHandoffTrigger() {
    return new Trigger(() -> (m_isHandOffReady && !DriverStation.isAutonomous()));
  }

  public void handoffComplete() {
    m_isHandOffReady = false;
    stop();

    if (Constants.kCurrentMode == Mode.SIM) {
      GamePieceVisualizer.setHasCoral(true);
    }
  }

  /** Only use this in simulation */
  public void setHandoffReady() {
    if (!GamePieceVisualizer.hasCoral()) {
      m_isHandOffReady = true;
      stop();
    }
  }

  public Command startFeederCmd() {
    Command cmd =
        new Command() {
          private boolean m_wasBeamBreakTriggered = false;
          private boolean m_isFinished = false;

          @Override
          public void initialize() {
            runFeeder(FeederConstants.kFeedSpeed);
            LEDSubsystem.getInstance().setStates(LEDStates.INTAKE_RUNNING);
            m_wasBeamBreakTriggered = false;
            m_isFinished = false;
            m_isHandOffReady = false;
          }

          @Override
          public void execute() {

            if (!m_wasBeamBreakTriggered && isBeamBreakTriggered()) {
              // feed slow
              runFeeder(FeederConstants.kFeedSlowSpeed);
              m_wasBeamBreakTriggered = true;
            } else if (m_wasBeamBreakTriggered && !isBeamBreakTriggered()) {
              stop();
              m_isHandOffReady = true;
              m_isFinished = true;
              LEDSubsystem.getInstance().setStates(LEDStates.IDLE);
            }
          }

          @Override
          public boolean isFinished() {
            return m_isFinished || (Constants.kCurrentMode == Mode.SIM && m_isHandOffReady);
          }
        };
    cmd.addRequirements(this);
    return new InstantCommand(() -> cmd.schedule());
  }

  public Command forceFeedCmd() {
    Command cmd =
        new Command() {
          @Override
          public void initialize() {
            runFeeder(FeederConstants.kFeedSpeed);
            m_isHandOffReady = false;
          }

          @Override
          public void end(boolean interrupted) {
            stop();
            LEDSubsystem.getInstance().setStates(LEDStates.IDLE);
          }
        };
    cmd.addRequirements(this);
    return cmd;
  }
}
