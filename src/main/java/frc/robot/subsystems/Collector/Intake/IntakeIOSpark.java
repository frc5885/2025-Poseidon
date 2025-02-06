
package frc.robot.subsystems.Collector.Intake;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;


import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

import frc.robot.Constants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOSpark implements IntakeIO {
    private SparkMax intakeMotor1;
    private SparkMax intakeMotor2;
    private RelativeEncoder intake1Encoder;
    private RelativeEncoder intake2Encoder;
    private Alert motor1disconnectedAlert;
    private Alert motor2disconnectedAlert;
    
    
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);

    public IntakeIOSpark(){
        intakeMotor1 = new SparkMax(Constants.IntakeId1, MotorType.kBrushed);
        intakeMotor2 = new SparkMax(Constants.IntakeId2, MotorType.kBrushed);

        motor1disconnectedAlert = new Alert("intake motor disconnected!", null);
        motor2disconnectedAlert = new Alert("intake motor disconnected!", null);

        intake1Encoder = intakeMotor1.getEncoder();
        intake2Encoder = intakeMotor2.getEncoder();


    SparkMaxConfig intake1Config = new SparkMaxConfig();
    intake1Config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(driveMotorCurrentLimit)
        .voltageCompensation(12.0);
    intake1Config
        .encoder
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    intake1Config
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        intakeMotor1,
        5,
        () ->
            intakeMotor1.configure(
                intake1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    // Reset neo relative encoder using absolute encoder position

    SparkMaxConfig intake2Config = intake1Config;

    intake2Config.follow(intakeMotor1);

    }


  public void updateInputs(IntakeIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(intakeMotor1, intake1Encoder ::getPosition, (value) -> inputs.positionRotation = value);
    ifOk(intakeMotor1, intake1Encoder::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        intakeMotor1,
        new DoubleSupplier[] {intakeMotor1::getAppliedOutput, intakeMotor1::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(intakeMotor1, intakeMotor1::getOutputCurrent, (value) -> inputs.current1Amps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

    sparkStickyFault = false;
    ifOk(intakeMotor2, intake2Encoder ::getPosition, (value) -> inputs.positionRotation = value);
    ifOk(intakeMotor2, intake2Encoder::getVelocity, (value) -> inputs.velocityRPM = value);
    ifOk(
        intakeMotor2,
        new DoubleSupplier[] {intakeMotor2::getAppliedOutput, intakeMotor2::getBusVoltage},
        (values) -> inputs.appliedVolts = values[0] * values[1]);
    ifOk(intakeMotor2, intakeMotor2::getOutputCurrent, (value) -> inputs.current2Amps = value);
    inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);
    }

  /** Run open loop at the specified voltage. */
  public void setVoltage(double volts) {
    intakeMotor1.setVoltage(volts);

  }
}
