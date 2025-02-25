package frc.robot.subsystems.Collector.Intake;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Collector.CollectorConstants.IntakeConstants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class IntakeIOSim implements IntakeIO {

  private double m_appliedVolts;
  private FlywheelSim m_flywheelSim;

  private final IntakeSimulation m_intakeSimulation;

  public IntakeIOSim(SwerveDriveSimulation driveTrain) {
    m_flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getNeo550(2), 0.001, IntakeConstants.kGearRatio),
            DCMotor.getNeo550(2));

    m_intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Coral",
            driveTrain,
            Inches.of(27.5),
            Inches.of(21),
            IntakeSimulation.IntakeSide.BACK,
            1);
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

  public void extendIntake() {
    m_intakeSimulation.startIntake();
  }

  public void retractIntake() {
    m_intakeSimulation.stopIntake();
  }

  public IntakeSimulation getMapleIntakeSimulation() {
    return m_intakeSimulation;
  }
}
